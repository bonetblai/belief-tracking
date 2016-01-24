/*
 *  Copyright (C) 2015-2016 Universidad Simon Bolivar
 *
 *  Permission is hereby granted to distribute this software for
 *  non-commercial research purposes, provided that this copyright
 *  notice is included with any such distribution.
 *
 *  THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 *  EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE
 *  SOFTWARE IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU
 *  ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 *
 *  Blai Bonet, bonet@ldc.usb.ve
 *
 */

#include <cassert>
#include <cstdlib>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <math.h>

#include <dai/alldai.h>
#include "../slam3/kappa.h"
#include "../slam3/inference.h"
#include "../slam3/edbp.h"
#include "../slam3/utils.h"

using namespace std;

// static members for inference algorithms
string Inference::inference_t::edbp_factors_fn_;
string Inference::inference_t::edbp_evid_fn_;
string Inference::inference_t::edbp_output_fn_;
vector<vector<int> > Inference::edbp_t::edbp_factor_indices_;

// static members for kappa handling
float kappa_t::epsilon_ = 0;
vector<float> kappa_t::powers_;

class tracking_t {
  protected:
    // game parameters
    int nrows_;
    int ncols_;
    int nmines_;
    bool noisy_;

    // statistics
    mutable int ngames_;
    mutable int nwins_;
    mutable int nguesses_;
    mutable int ndecisions_;
    mutable int ninferences_;
    mutable float elapsed_time_;
 
  public:
    tracking_t(int nrows, int ncols, int nmines, bool noisy)
      : nrows_(nrows), ncols_(ncols), nmines_(nmines), noisy_(noisy) {
    }
    virtual ~tracking_t() { }

    bool is_flag_action(int action) const {
        return action < nrows_ * ncols_ ? true : false;
    }
    int get_cell(int action) const {
        return is_flag_action(action) ? action : action - (nrows_ * ncols_);
    }

    void initialize_stats() const {
        ngames_ = 0;
        nwins_ = 0;
        nguesses_ = 0;
        ndecisions_ = 0;
        ninferences_ = 0;
        elapsed_time_ = 0;
    }
    void print_stats(ostream &os) const {
          os << "stats: #games=" << ngames_
             << ", #wins=" << nwins_
             << ", %win=" << float(nwins_) / float(ngames_)
             << ", #guesses=" << nguesses_
             << ", #decisions=" << ndecisions_
             << ", #inferences=" << ninferences_
             << ", etime=" << elapsed_time_
             << ", etime/game=" << elapsed_time_ / float(ngames_)
             << ", etime/decision=" << elapsed_time_ / float(ndecisions_)
             << ", etime/inference=" << elapsed_time_ / float(ninferences_)
             << endl;
    }

    void increase_elapsed_time(float time) {
        elapsed_time_ += time;
    }
    void increase_wins() {
        ++nwins_;
    }

    virtual string id() const = 0;
    virtual void reset() = 0;
    virtual void update(bool flag_action, int cell, int obs) = 0;
    virtual int get_action() const = 0;
};

// standard belief tracking for minesweeper
class pbt_t : public tracking_t {
    // variables, factors and centers for each beam
    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;
    vector<int> centers_;

    // computation of marginals in factor model
    mutable vector<int> indices_for_updated_factors_;
    mutable vector<dai::Factor> marginals_;

    // inference algorithm and parameters
    Inference::inference_t inference_;

    // arc consistency
    bool use_ac3_;
    int iterated_level_;
    bool inverse_check_;
    //CHECK kappa_arc_consistency_t kappa_csp_;

    // variables for game play
    set<int> plays_;
    int nflags_;

    // for action selection
    mutable float best_prob_for_open_;
    mutable float best_prob_for_flag_;
    mutable vector<int> best_for_open_;
    mutable vector<int> best_for_flag_;

  public:
    pbt_t(int nrows, int ncols, int nmines, bool noisy, const multimap<string, string> parameters)
      : tracking_t(nrows, ncols, nmines, noisy) {
        // create binary variables for each cell in the grid
        variables_ = vector<dai::Var>(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc )
            variables_[loc] = dai::Var(loc, 2);

        // create one factor for each location. The variables
        // in the factor are the variables for the location
        // surrounding the factor, including the variable
        // for the "center" location. Also set up the center
        // for each factor.
        centers_ = vector<int>(nrows_ * ncols_);
        factors_ = vector<dai::Factor>(nrows_ * ncols_);
        marginals_ = vector<dai::Factor>(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            int row = loc / ncols_, col = loc % ncols_;
            vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    if( (dr == 0) && (dc == 0) ) centers_[loc] = vars.size();
                    vars.push_back(variables_[nr * ncols_ + nc]);
                }
            }
            dai::VarSet varset(vars.begin(), vars.end());
            factors_[loc] = dai::Factor(varset);
            marginals_[loc] = dai::Factor(variables_[loc]);
            //cout << "Factor[row=" << row << ",col=" << col << "]=" << p << endl;
        }
        cout << "pbt: factor graph:"
             << " #variables=" << variables_.size()
             << ", #factors=" << factors_.size()
             << endl;

        multimap<string, string>::const_iterator it = parameters.find("inference");
        if( it != parameters.end() ) {
            inference_.set_inference_algorithm(it->second, "MAR", true);
            if( inference_.algorithm() == "iterated-ac3" ) {
                const dai::PropertySet &options = inference_.options();
                use_ac3_ = true;
                if( options.hasKey("level") )
                    iterated_level_ = int(options.getStringAs<size_t>("level"));
                if( options.hasKey("inverse-check") )
                    inverse_check_ = options.getStringAs<bool>("inverse-check");
                //CHECK kappa_csp_.set_iterated_level(iterated_level_);
                //CHECK kappa_csp_.set_inverse_check(inverse_check_);
            } else {
                use_ac3_ = false;
                inference_.create_and_initialize_algorithm(factors_);
            }
        }
        it = parameters.find("edbp-max-iter");
        if( it != parameters.end() )
            inference_.edbp_max_iter_ = strtoul(it->second.c_str(), 0, 0);
    }
    virtual ~pbt_t() { }

    virtual string id() const {
        string id_str;
        id_str = string("pbt()"); // CHECK
        return id_str;
    }

    // reset all factors and game-play variables
    virtual void reset() {
        indices_for_updated_factors_.clear();
        indices_for_updated_factors_.reserve(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / (1 << factor.vars().size());
            for( int j = 0; j < (1 << factor.vars().size()); ++j )
                factor.set(j, p);
            indices_for_updated_factors_.push_back(loc);
        }
        plays_.clear();
        nflags_ = 0;
        best_for_open_.clear();
        best_for_flag_.clear();
        ++ngames_;
        //CHECK inference_.destroy_inference_algorithm();
    }

    // update for obtained obs for cell
    void update(bool flag_action, int cell, int obs) {
        assert(plays_.find(cell) == plays_.end());
        plays_.insert(cell);
        //int row = cell / ncols_, col = cell % ncols_;
        //cout << "pbt: flag=" << flag_action
        //     << ", cell=" << cell << ":(" << col << "," << row << ")"
        //     << ", obs=" << obs << endl;
        if( !flag_action ) {
            int center = centers_[cell];
            //cout << "center=" << center << ", var=" << factors_[cell].vars().var(center) << endl;
            dai::Factor &factor = factors_[cell];
            for( int j = 0; j < int(factor.nrStates()); ++j ) {
                int popcount = __builtin_popcount(j);
                if( !noisy_ ) {
                    if( popcount != obs ) factor.set(j, 0);
                } else {
                    // noisy update
                    if( popcount == obs ) {
                        factor.set(j, factor[j] * .95);
                    } else if( (popcount == obs - 1) || (popcount == obs + 1) ) {
                        factor.set(j, factor[j] * .05);
                    } else {
                        factor.set(j, 0);
                    }
                }
                if( (j & (1 << center)) != 0 ) factor.set(j, 0);
            }
            indices_for_updated_factors_.push_back(cell);

            // if obs is 0 and not noisy, neighboring cells don't have mines
            if( !noisy_ && (obs == 0) ) {
                if( best_prob_for_open_ < 1 ) {
                    best_prob_for_open_ = 1;
                    best_for_open_.clear();
                }
                int row = cell / ncols_, col = cell % ncols_;
                for( int dr = -1; dr < 2; ++dr ) {
                    int nr = row + dr;
                    if( (nr < 0) || (nr >= nrows_) ) continue;
                    for( int dc = -1; dc < 2; ++dc ) {
                        int nc = col + dc;
                        if( (dr == 0) && (dc == 0) ) continue;
                        if( (nc < 0) || (nc >= ncols_) ) continue;
                        int nloc = nr * ncols_ + nc;
                        //cout << "adding (" << nc << "," << nr << ")" << endl;
                        best_for_open_.push_back(nloc);
                    }
                }
            }
        } else {
            ++nflags_;
        }
    }

    // recommend action using information in the marginals
    virtual int get_action() const {
        ++ndecisions_;
        int action = -1;
        if( (best_prob_for_open_ == 1) && !best_for_open_.empty() ) {
            int cell = -1;
            while( (cell == -1) && !best_for_open_.empty() ) {
                int index = lrand48() % best_for_open_.size();
                int candidate = best_for_open_[index];
                best_for_open_[index] = best_for_open_.back();
                best_for_open_.pop_back();
                if( plays_.find(candidate) == plays_.end() )
                    cell = candidate;
            }
            if( cell != -1 ) action = cell + (nrows_ * ncols_);
        }

        if( (action == -1) && (best_prob_for_flag_ == 1) && !best_for_flag_.empty() ) {
            int cell = -1;
            while( (cell == -1) && !best_for_flag_.empty() ) {
                int index = lrand48() % best_for_flag_.size();
                int candidate = best_for_flag_[index];
                best_for_flag_[index] = best_for_flag_.back();
                best_for_flag_.pop_back();
                if( plays_.find(candidate) == plays_.end() )
                    cell = candidate;
            }
            if( cell != -1 ) action = cell;
        }

        if( action == -1 ) {
            calculate_marginals();
            best_for_open_.clear();
            best_for_flag_.clear();
            best_prob_for_open_ = 0;
            best_prob_for_flag_ = 0;
            for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
                if( plays_.find(loc) != plays_.end() ) continue; // cell had been already played
                const dai::Factor &marginal = marginals_[loc];
                assert(marginal.nrStates() == 2);
                if( best_for_open_.empty() || (marginal[0] >= best_prob_for_open_) ) {
                    if( best_for_open_.empty() || (marginal[0] > best_prob_for_open_) ) {
                        best_prob_for_open_ = marginal[0];
                        best_for_open_.clear();
                    }
                    best_for_open_.push_back(loc);
                }
                if( (nflags_ < nmines_) && (best_for_flag_.empty() || (marginal[1] >= best_prob_for_flag_)) ) {
                    if( best_for_flag_.empty() || (marginal[1] > best_prob_for_flag_) ) {
                        best_prob_for_flag_ = marginal[1];
                        best_for_flag_.clear();
                    }
                    best_for_flag_.push_back(loc);
                }
            }

            cout << "best for open: p=" << best_prob_for_open_ << ", sz=" << best_for_open_.size() << ", loc=";
            for( int i = 0; i < int(best_for_open_.size()); ++i )
                cout << best_for_open_[i] << ",";
            cout << endl << "best for flag: p=" << best_prob_for_flag_ << ", sz=" << best_for_flag_.size() << ", loc=";
            for( int i = 0; i < int(best_for_flag_.size()); ++i )
                cout << best_for_flag_[i] << ",";
            cout << endl;

            // prioritize open over flag actions
            if( !best_for_open_.empty() && (best_prob_for_open_ >= best_prob_for_flag_) ) {
                nguesses_ += best_prob_for_open_ < 1;
                int index = lrand48() % best_for_open_.size();
                int cell = best_for_open_[index];
                best_for_open_[index] = best_for_open_.back();
                best_for_open_.pop_back();
                action = cell + (nrows_ * ncols_);
            } else if( !best_for_flag_.empty() && (best_prob_for_open_ < best_prob_for_flag_) ) {
                nguesses_ += best_prob_for_flag_ < 1;
                int index = lrand48() % best_for_flag_.size();
                int cell = best_for_flag_[index];
                best_for_flag_[index] = best_for_flag_.back();
                best_for_flag_.pop_back();
                action = cell;
            }
        }

        assert(action != -1);
        return action;
    }

  protected:
    void calculate_marginals(bool print_marginals = false) const {
        ++ninferences_;
        inference_.calculate_marginals(variables_,
                                       indices_for_updated_factors_,
                                       factors_,
                                       marginals_,
                                       0, // not used because type = MAR
                                       print_marginals);
    }
};

struct cell_t {
    bool mine_;
    int nmines_;
    cell_t() : mine_(false), nmines_(0) { }
};

class minefield_t {
  protected:
    int nrows_;
    int ncols_;
    int ncells_;
    int nmines_;
    int num_remaining_mines_;

    vector<cell_t> cells_;

  public:
    minefield_t(int nrows, int ncols, int nmines)
      : nrows_(nrows), ncols_(ncols), ncells_(nrows_ * ncols_), nmines_(nmines) {
    }
    ~minefield_t() { }

    void sample(int initial_cell) {
        // do not place a mine at the initial cell or surrounding cells
        // calculate places where a mine can be placed
        vector<int> available_cells(ncells_, 0);
        set<int> forbidden;
        do {
            forbidden.clear();
            available_cells = vector<int>(ncells_, 0);
            for( int i = 0; i < ncells_; ++i )
                available_cells[i] = i;

            int r = initial_cell / ncols_, c = initial_cell % ncols_;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = r + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = c + dc;
                    int cell = nr * ncols_ + nc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    forbidden.insert(-(1 + cell));
                }
            }
            for( set<int>::iterator it = forbidden.begin(); it != forbidden.end(); ++it ) {
                int pos = -(*it) - 1;
                available_cells[pos] = available_cells.back();
                available_cells.pop_back();
            }
        } while( (int)available_cells.size() < nmines_ );

        // available_cells contains the cells where mines can be placed.
        // Place mines in random cells. For each placed mines, increase
        // counter of surronding mines in surrounding cells.
        cells_ = vector<cell_t>(ncells_);
        for( int num_placed_mines = 0; num_placed_mines < nmines_; ++num_placed_mines ) {
            int pos = lrand48() % available_cells.size();
            int cell = available_cells[pos];
            available_cells[pos] = available_cells.back();
            available_cells.pop_back();
            assert(!cells_[cell].mine_);
            cells_[cell].mine_ = true;
            int r = cell / ncols_, c = cell % ncols_;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = r + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = c + dc;
                    int ncell = nr * ncols_ + nc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    if( cell == ncell ) continue;
                    ++cells_[ncell].nmines_;
                }
            }
        }
        num_remaining_mines_ = nmines_;
    }

    int apply_action(const tracking_t &tracker, int action, bool verbose) {
        int cell = tracker.get_cell(action);
        if( tracker.is_flag_action(action) ) {
            return flag_cell(cell, verbose);
        } else {
            return open_cell(cell, verbose);
        }
    }

    bool is_mine(int cell) const {
        return cells_[cell].mine_;
    }

    void print(ostream &os, bool formatted = false) const {
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                if( cells_[cell].mine_ )
                    os << " *";
                else
                    os << " " << cells_[cell].nmines_;
            }
            if( formatted ) os << endl;
        }
    }

  protected:
    int num_remaining_mines() const {
        return num_remaining_mines_;
    }

    int flag_cell(int cell, bool verbose) {
        int r = cell / ncols_, c = cell % ncols_;
        if( verbose ) cout << "flag_cell(" << c << "," << r << "): mine=" << (cells_[cell].mine_ ? 1 : 0) << endl;
        num_remaining_mines_ -= cells_[cell].mine_ ? 1 : 0;
        return -1;
    }

    int open_cell(int cell, bool verbose) {
        int r = cell / ncols_, c = cell % ncols_;
        if( verbose ) cout << "open_cell(" << c << "," << r << "): mine=" << (cells_[cell].mine_ ? 1 : 0) << ", #mines=" << cells_[cell].nmines_ << endl;
        return cells_[cell].mine_ ? 9 : cells_[cell].nmines_;
    }
};

#if 0
// divergence measures between distributions
pair<float, bool> kl_divergence(const vector<float> &P, const vector<float> &Q) {
    assert(P.size() == Q.size());
    float kl = 0;
    for( size_t i = 0; i < P.size(); ++i ) {
        float p = P[i];
        float q = Q[i];
        if( (q == 0) && (p != 0) ) return make_pair(kl, false);
        if( p == 0 ) continue;
        kl += p * (log(p) - log(q));
    }
    return make_pair(kl, true);
}

pair<float, bool> js_divergence(const vector<float> &P, const vector<float> &Q) {
    assert(P.size() == Q.size());
    vector<float> M(P.size(), 0);
    for( size_t i = 0; i < M.size(); ++i )
        M[i] = (P[i] + Q[i]) / 2;
    pair<float, bool> kl1 = kl_divergence(P, M);
    pair<float, bool> kl2 = kl_divergence(Q, M);
    assert(kl1.second && kl2.second);
    return make_pair((kl1.first + kl2.first) / 2, true);
}
#endif

void usage(ostream &os) {
    os << endl
       << "Usage: mines [{-t | --ntrials} <ntrials>]" << endl
       << "             [{-r | --nrows} <nrows>]" << endl
       << "             [{-c | --ncols} <ncols>]" << endl
       << "             [{-m | --nmines} <nmines>]" << endl
       << "             [{-s | --seed} <seed>]" << endl
       << "             [{-v | --verbose}]" << endl
       << "             [{-? | --help}]" << endl
       << endl
       << "where <ntrials> is a non-negative integer telling the number of games to" << endl
       << "play (default is 1), <nrows> and <ncols> are positive integers telling" << endl
       << "the dimensions of the minefield (default is 16x16), <nmines> is a positive" << endl
       << "integer telling the number of hidden mines in the minefield (default is 40)," << endl
       << "and <seed> is an integer for setting the seed of the random number generator" << endl
       << "(default is 0)." << endl
       << endl
       << "For example," << endl
       << endl
       << "  ./mines -r 16 -c 16 -m 40 -t 100" << endl
       << endl
       << "performs an experiment consisting of 100 trials on a 16x16 minefield" << endl
       << "with 40 mines." << endl
       << endl;
}

void finalize() {
}

int main(int argc, const char **argv) {
    int ntrials = 1;
    int nrows = 16;
    int ncols = 16;
    int nmines = 40;
    int seed = 0;
    bool verbose = false;
    float epsilon_for_kappa = 0.1;
    string tmp_path = "";
    vector<string> tracker_strings;

    // parse arguments
    for( --argc, ++argv; (argc > 0) && (**argv == '-'); --argc, ++argv ) {
        if( !strcmp(argv[0], "--tmp-path") ) {
            tmp_path = argv[1];
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-t") || !strcmp(argv[0], "--ntrials") ) {
            ntrials = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-r") || !strcmp(argv[0], "--nrows") ) {
            nrows = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-c") || !strcmp(argv[0], "--ncols") ) {
            ncols = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-m") || !strcmp(argv[0], "--nmines") ) {
            nmines = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strncmp(argv[0], "--tracker=", 10) ) {
            string tracker(&argv[0][10]);
            Utils::tokenize(tracker, tracker_strings);
        } else if( !strcmp(argv[0], "-v") || !strcmp(argv[0], "--verbose") ) {
            verbose = true;
        } else if( !strcmp(argv[0], "-?") || !strcmp(argv[0], "--help") ) {
            usage(cout);
            exit(-1);
        } else {
            cout << "error: unexpected argument: " << argv[0] << endl;
        }
    }

    // set seed
    Utils::set_seed(seed);
    cout << "# seed=" << seed << endl;

    // set static members
    if( !tmp_path.empty() && (tmp_path.back() != '/') ) tmp_path += '/';
    //Inference::edbp_t::initialize();
    //Inference::inference_t::initialize_edbp(tmp_path);
    kappa_t::initialize(epsilon_for_kappa, 10);

    // tracking algorithms
    vector<tracking_t*> trackers;
    for( size_t i = 0; i < tracker_strings.size(); ++i ) {
        tracking_t *tracker = 0;
        const string &name = tracker_strings[i];
        string short_name;
        string parameter_str;
        multimap<string, string> parameters;
        Utils::split_request(name, short_name, parameter_str);
        Utils::tokenize(parameter_str, parameters);
        if( short_name == "pbt" ) {
            tracker = new pbt_t(nrows, ncols, nmines, false, parameters);
        } else {
            cerr << "warning: unrecognized tracking algorithm '" << name << "'" << endl;
        }
        if( tracker != 0 ) trackers.push_back(tracker);
    }

    // check that there is something to do
    if( trackers.empty() ) {
        cout << "warning: no tracker specified. Terminating..." << endl;
        finalize();
        return 0;
    }

    // print identity of trackers
    for( size_t i = 0; i < trackers.size(); ++i )
        cout << "# tracker[" << i << "].id=\"" << trackers[i]->id() << "\"" << endl;
    
    // initialize stats for trackers
    for( size_t i = 0; i < trackers.size(); ++i )
        trackers[i]->initialize_stats();

    // run for the specified number of trials
    for( int trial = 0; trial < ntrials; ) {
        for( int i = 0; i < int(trackers.size()); ++i ) {
            minefield_t minefield(nrows, ncols, nmines);
            tracking_t &tracker = *trackers[i];
            tracker.reset();
            float start_time = Utils::read_time_in_seconds();

            bool win = true;
            vector<pair<int,int> > execution(nrows * ncols);
            for( int play = 0; play < nrows * ncols; ++play ) {
                int action = tracker.get_action();
                bool is_flag_action = tracker.is_flag_action(action);
                int cell = tracker.get_cell(action);
                cout << "Play: n=" << play
                     << ", type=" << (is_flag_action ? "FLAG" : "OPEN")
                     << ", cell=" << cell << ":(" << cell % ncols << "," << cell / ncols << ")"
                     << flush;

                // if this is first play, then it must be an open action. The minefield
                // gets sampled using the action's cell.
                if( play == 0 ) {
                    assert(!tracker.is_flag_action(action));
                    int cell = tracker.get_cell(action);
                    minefield.sample(cell);
                    assert(!minefield.is_mine(cell));
                    cout << ", obs=0" << endl << endl;
                    minefield.print(cout, true);
                    cout << endl;
                }

                // obtain observation for this action. If first play, observation
                // must be zero.
                int obs = minefield.apply_action(tracker, action, verbose);
                assert((obs == 0) || (play > 0));
                if( play > 0 ) cout << ", obs=" << obs << endl;
                if( obs == 9 ) {
                    cout << "**** BOOM!!! ****" << endl;
                    win = false;
                    break;
                }

                // update execution and agent's belief 
                execution[play] = make_pair(action, obs);

                // update agent's belief
                tracker.update(tracker.is_flag_action(action), tracker.get_cell(action), obs);
            }
            tracker.increase_elapsed_time(Utils::read_time_in_seconds() - start_time);

            if( win ) tracker.increase_wins();
            tracker.print_stats(cout);
            ++trial;
        }
    }
    return 0;
}

