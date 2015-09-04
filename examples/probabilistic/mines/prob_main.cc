/*
 *  Copyright (C) 2014 Universidad Simon Bolivar
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
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <stdlib.h>
#include <math.h>

#include <dai/alldai.h>

extern "C" {
#include "c_api.h"
};

using namespace std;


struct cell_t {
    bool mine_;
    int nmines_;
    cell_t() : mine_(false), nmines_(0) { }
};

struct minefield_t {
    int nrows_;
    int ncols_;
    int ncells_;
    int nmines_;
    int num_remaining_mines_;

    vector<cell_t> cells_;

    minefield_t(int nrows, int ncols, int nmines)
      : nrows_(nrows), ncols_(ncols), ncells_(nrows_ * ncols_), nmines_(nmines) {
    }

    int num_remaining_mines() const { return num_remaining_mines_; }

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
                    forbidden.insert(-cell);
                }
            }
            for( set<int>::iterator it = forbidden.begin(); it != forbidden.end(); ++it ) {
                int pos = -(*it);
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

    int apply_action(int action, bool verbose) {
        int cell = agent_get_cell(action);
        if( agent_is_flag_action(action) ) {
            return flag_cell(cell, verbose);
        } else {
            return open_cell(cell, verbose);
        }
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
};

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

// probabilistic belief tracking for minesweeper
struct ms_pbt_t {
    // grid dimension, num of mines, and noisy flag
    int nrows_;
    int ncols_;
    int nmines_;
    bool noisy_;

    // variables, factors, and centers for each beam.
    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;
    vector<int> centers_;

    // for belief inference
    mutable dai::Factor joint_;
    mutable dai::FactorGraph jt_fg_;
    mutable dai::JTree jt_;
    mutable dai::FactorGraph approx_inference_fg_;
    mutable dai::InfAlg *approx_inference_algorithm_;

    // variables for game play
    set<int> plays_;
    int nflags_;

    ms_pbt_t(int nrows, int ncols, int nmines, bool noisy = false)
      : nrows_(nrows), ncols_(ncols), nmines_(nmines), noisy_(noisy) {
        // create binary variables for each cell in the grid
        variables_ = vector<dai::Var>(nrows_ * ncols_);
        for( int i = 0; i < nrows_ * ncols_; ++i )
            variables_[i] = dai::Var(i, 2);

        // create factors: one factor per cell.
        // Centers ???
        centers_ = vector<int>(nrows_ * ncols_);
        factors_ = vector<dai::Factor>(nrows_ * ncols_);
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            int row = i / ncols_, col = i % ncols_;
            vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    if( (dr == 0) && (dc == 0) ) centers_[i] = vars.size();
                    vars.push_back(variables_[nr * ncols_ + nc]);
                }
            }
            dai::VarSet varset(vars.begin(), vars.end());
            factors_[i] = dai::Factor(varset);
            //cout << "Factor[row=" << row << ",col=" << col << "]=" << p << endl;
        }
        cout << "pbt: factor graph:"
             << " #variables=" << variables_.size()
             << ", #factors=" << factors_.size()
             << endl;
    }

    // reset all factors and game-play variables
    void reset() {
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            dai::Factor &factor = factors_[i];
            float p = 1.0 / (1 << factor.vars().size());
            for( int j = 0; j < (1 << factor.vars().size()); ++j )
                factor.set(j, p);
        }
        plays_.clear();
        nflags_ = 0;
        approx_inference_algorithm_ = 0;
    }

    // update factors for obtained obs for cell
    void update(bool flag_action, int cell, int obs) {
        assert(plays_.find(cell) == plays_.end());
        plays_.insert(cell);
        int row = cell / ncols_, col = cell % ncols_;
        //cout << "pbt: flag=" << flag_action
        //     << ", cell=" << cell << ":(" << col << "," << row << ")"
        //     << ", obs=" << obs << endl;
        if( !flag_action ) {
            int center = centers_[cell];
            //cout << "center=" << center << ", var=" << factors_[cell].vars().var(center) << endl;
            dai::Factor &factor = factors_[cell];
            const dai::VarSet &vars = factor.vars();
            for( int j = 0; j < (1 << vars.size()); ++j ) {
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
        } else {
            ++nflags_;
        }
    }

    // compute full joint by multiplying factors (exponential)
    void compute_joint() const {
        joint_ = dai::Factor();
        for( int i = 0; i < nrows_ * ncols_; ++i )
            joint_ *= factors_[i];
        joint_.normalize();
    }

    // return the marginal distribution of the full joint over a cell variable
    pair<float, float> joint_marginal(int cell) const {
        return make_pair(float(joint_.marginal(dai::VarSet(variables_[cell]))[0]), float(joint_.marginal(dai::VarSet(variables_[cell]))[1]));
    }
    void joint_marginal(int cell, dai::Factor &marginal) const {
        marginal = joint_.marginal(dai::VarSet(variables_[cell]));
    }

    // use junction tree algorithm to compute the exact marginals for each factor
    void apply_junction_tree() const {
        dai::PropertySet opts;
        jt_fg_ = dai::FactorGraph(factors_);
        //size_t maxstates = 1e8;
        //dai::boundTreewidth(jt_fg_, &dai::eliminationCost_MinFill, maxstates);
        jt_ = dai::JTree(jt_fg_, opts("updates", string("HUGIN")));
        jt_.init();
        jt_.run();
    }
    void apply_junction_tree(vector<float> &P) const {
        P.clear();
        apply_junction_tree();
        float mass = 0;
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            P.push_back(jt_.belief(jt_fg_.var(i))[0]);
            mass = P.back();
        }
        for( int i = 0; i < nrows_ * ncols_; ++i ) P[i] /= mass;
    }
    pair<float, float> jt_marginal(int cell) const {
        return make_pair(float(jt_.belief(jt_fg_.var(cell))[0]), float(jt_.belief(jt_fg_.var(cell))[1]));
    }

    void apply_approx_inference() const {
        dai::PropertySet opts;
        approx_inference_fg_ = dai::FactorGraph(factors_);
        delete approx_inference_algorithm_;
        //approx_inference_algorithm_ = new dai::BP(approx_inference_fg_, opts("updates", string("SEQRND"))("logdomain", true)("tol", 1e-9)("maxiter", (size_t)10000));
        //approx_inference_algorithm_ = new dai::Gibbs(approx_inference_fg_, opts("maxiter", (size_t)10000)("burnin", (size_t)100)("restart", (size_t)10000));
        approx_inference_algorithm_ = new dai::HAK(approx_inference_fg_, opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(1)));
        approx_inference_algorithm_->init();
        approx_inference_algorithm_->run();
    }
    void apply_approx_inference(vector<float> &P) const {
        P.clear();
        apply_approx_inference();
        float mass = 0;
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            P.push_back(approx_inference_algorithm_->belief(approx_inference_fg_.var(i))[0]);
            mass = P.back();
        }
        for( int i = 0; i < nrows_ * ncols_; ++i ) P[i] /= mass;
    }
    pair<float, float> approx_inference_marginal(int cell) const {
        return make_pair(float(approx_inference_algorithm_->belief(approx_inference_fg_.var(cell))[0]),
                         float(approx_inference_algorithm_->belief(approx_inference_fg_.var(cell))[1]));
    }

    // recommend action using information in the marginals
    int agent_get_action(pair<float, float> (ms_pbt_t::*marginal)(int) const) const {
        float best_prob_for_open = 0, best_prob_for_flag = 0;
        vector<int> best_for_open, best_for_flag;
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            if( plays_.find(i) != plays_.end() ) continue; // cell had been already played
            pair<float, float> p((this->*marginal)(i));    // get marginal on cell
            if( best_for_open.empty() || (p.first >= best_prob_for_open) ) {
                if( best_for_open.empty() || (p.first > best_prob_for_open) ) {
                    best_prob_for_open = p.first;
                    best_for_open.clear();
                }
                best_for_open.push_back(i);
            }
            if( (nflags_ < nmines_) && (best_for_flag.empty() || (p.second >= best_prob_for_flag)) ) {
                if( best_for_flag.empty() || (p.second > best_prob_for_flag) ) {
                    best_prob_for_flag = p.second;
                    best_for_flag.clear();
                }
                best_for_flag.push_back(i);
            }
        }

        // prioritize open over flag actions
        if( !best_for_open.empty() && (best_prob_for_open >= best_prob_for_flag) ) {
            int cell = best_for_open[lrand48() % best_for_open.size()];
            return cell + (nrows_ * ncols_);
        } else if( !best_for_flag.empty() && (best_prob_for_open < best_prob_for_flag) ) {
            int cell = best_for_flag[lrand48() % best_for_flag.size()];
            return cell;
        } else if( !best_for_open.empty() ) {
            int cell = best_for_open[lrand48() % best_for_open.size()];
            return cell + (nrows_ * ncols_);
        } else if( nflags_ < nmines_ ) {
            assert(!best_for_flag.empty());
            int cell = best_for_flag[lrand48() % best_for_flag.size()];
            return cell;
        } else {
            assert(0);
            return 0;
        }
    }
    bool is_flag_action(int action) {
        return action < nrows_ * ncols_ ? true : false;
    }
    int agent_get_cell(int action) {
        return is_flag_action(action) ? action : action - (nrows_ * ncols_);
    }

    void print_marginals(ostream &os, pair<float, float> (ms_pbt_t::*marginal)(int) const, int prec = 2) {
        int old_prec = os.precision();
        os << setprecision(prec);
        for( int row = 0; row < nrows_; ++row ) {
            os << "|";
            for( int col = 0; col < ncols_; ++col ) {
                int cell = row * ncols_ + col;
                pair<float, float> p = (this->*marginal)(cell);
                os << " " << setw(4) << 1 - p.first << (plays_.find(cell) != plays_.end() ? "*" : " ") << "|";
            }
            os << endl;
        }
        os << setprecision(old_prec);
    }
};

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

int main(int argc, const char **argv) {
    int ntrials = 1;
    int nrows = 16;
    int ncols = 16;
    int nmines = 40;
    int seed = 0;
    bool verbose = false;

    --argc;
    ++argv;
    while( (argc > 0) && (**argv == '-') ) {
        if( !strcmp(argv[0], "-t") || !strcmp(argv[0], "--ntrials") ) {
            ntrials = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-r") || !strcmp(argv[0], "--nrows") ) {
            nrows = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-c") || !strcmp(argv[0], "--ncols") ) {
            ncols = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-m") || !strcmp(argv[0], "--nmines") ) {
            nmines = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-v") || !strcmp(argv[0], "--verbose") ) {
            verbose = true;
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-?") || !strcmp(argv[0], "--help") ) {
            usage(cout);
            exit(-1);
        } else {
            cout << "error: unexpected argument: " << argv[0] << endl;
            --argc;
            ++argv;
        }
    }

    // set seed
    unsigned short seeds[3];
    seeds[0] = seeds[1] = seeds[2] = seed;
    seed48(seeds);

    // create distributions
    ms_pbt_t pbt(nrows, ncols, nmines, false);
    //ms_pbt_t pbt(nrows, ncols, nmines, true);

    // run for the specified number of trials
    for( int trial = 0; trial < ntrials; ) {
        minefield_t minefield(nrows, ncols, nmines);
        agent_initialize(nrows, ncols, nmines);
        pbt.reset();

#if 1
        // compute and print exact marginals via junction tree
        vector<float> P;
        cout << "Marginals (exact: junction tree):" << endl;
        pbt.apply_junction_tree(P);
        pbt.print_marginals(cout, &ms_pbt_t::jt_marginal);
#endif
#if 0
        // approximate and print marginals
        vector<float> Q;
        cout << "Marginals (approx. inference):" << endl;
        pbt.apply_approx_inference(Q);
        pbt.print_marginals(cout, &ms_pbt_t::approx_inference_marginal);
#endif

        bool win = true;
        vector<pair<int,int> > execution(nrows * ncols);
        for( int play = 0; play < nrows * ncols; ++play ) {
            //int is_guess = 2;
            //int action = agent_get_action(&is_guess);
            int action = pbt.agent_get_action(&ms_pbt_t::jt_marginal);
            //int action = pbt.agent_get_action(&ms_pbt_t::approx_inference_marginal);

            bool is_flag_action = pbt.is_flag_action(action);
            int cell = pbt.agent_get_cell(action);
            cout << "Play: n=" << play
                 << ", type=" << (is_flag_action ? "FLAG" : "OPEN")
                 << ", cell=" << cell << ":(" << cell % ncols << "," << cell / ncols << ")"
                 << flush;

            // if this is first play, then it must be an open action. The minefield
            // gets sampled using the action's cell.
            if( play == 0 ) {
                assert(!pbt.is_flag_action(action));
                int cell = agent_get_cell(action);
                minefield.sample(cell);
                assert(!minefield.cells_[cell].mine_);
                cout << endl << endl;
                minefield.print(cout, true);
                cout << endl;
            }

            // obtain observation for this action. If first play, observation
            // must be zero.
            int obs = minefield.apply_action(action, verbose);
            assert((obs == 0) || (play > 0));
            if( play > 0 ) cout << ", obs=" << obs << endl;
            if( obs == 9 ) {
                cout << "BOOM" << endl;
                win = false;
                break;
            }

            // update execution and agent's belief 
            execution[play] = make_pair(action, obs);

#if 0
            if( (play > 0) && !agent_is_flag_action(action) && (pbt.joint_marginal(agent_get_cell(action)) != 1.0) ) {
                cout << "pbt: INSECURE ACTION: open cell ("
                     << agent_get_cell(action) / ncols << ","
                     << agent_get_cell(action) % ncols << ")"
                     << endl;
            }
#endif

            // update agent's belief
            agent_update_state(agent_is_flag_action(action), agent_get_cell(action), obs);
            pbt.update(pbt.is_flag_action(action), agent_get_cell(action), obs);

#if 1
            // compute and print exact marginals via junction tree
            vector<float> P;
            cout << "Marginals (exact: junction tree):" << endl;
            pbt.apply_junction_tree(P);
            pbt.print_marginals(cout, &ms_pbt_t::jt_marginal);
#endif
#if 0
            // approximate and print marginals
            vector<float> Q;
            cout << "Marginals (approx. inference):" << endl;
            pbt.apply_approx_inference(Q);
            pbt.print_marginals(cout, &ms_pbt_t::approx_inference_marginal);
#endif
#if 0
            // calculate KL-divergence
            pair<float, bool> kl1 = kl_divergence(P, Q);
            pair<float, bool> kl2 = kl_divergence(Q, P);
            pair<float, bool> js = js_divergence(P, Q);
            cout << "KL-divergence: v1=" << kl1.first << ", s1=" << kl1.second << ", v2=" << kl2.first << ", s2=" << kl2.second << ", R=" << (kl1.first * kl2.first) / (kl1.first + kl2.first) << ", J=" << js.first << endl;
#endif
        }
        if( win ) {
            agent_declare_win(true);
        } else {
            agent_declare_lose(true);
        }
        ++trial;
    }
    agent_finalize();
    return 0;
}

