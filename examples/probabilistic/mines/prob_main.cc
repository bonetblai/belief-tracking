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

    // inference type and tmp filenames
    int inference_type_;
    string edbp_tmp_fn_;
    string edbp_evid_fn_;

    // variables, factors, and centers for each beam.
    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;
    vector<int> centers_;

    // for belief inference
    mutable dai::Factor full_joint_;
    mutable dai::InfAlg *inference_algorithm_;
    mutable vector<dai::Factor> marginals_;
    mutable bool need_to_recalculate_marginals_;

    // variables for game play
    set<int> plays_;
    int nflags_;

    ms_pbt_t(int nrows, int ncols, int nmines, int inference_type = 0, bool noisy = false)
      : nrows_(nrows), ncols_(ncols), nmines_(nmines),
        noisy_(noisy), inference_type_(inference_type) {
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

        // computation of marginal
        need_to_recalculate_marginals_ = true;

        // create inference algorithm
        create_and_initialize_inference_algorithm();

        // create tmp file for edbp inference
        if( inference_type_ == 1 ) {
            edbp_tmp_fn_ = "dummy.uai";
            edbp_evid_fn_ = "dummy.evid";
        }
    }
    ~ms_pbt_t() {
        destroy_inference_algorithm();
    }

    void create_and_initialize_inference_algorithm() {
        dai::FactorGraph factor_graph(factors_);

#if 0
        // compute junction tree using min-fill heuristic
        size_t maxstates = 1e6;
        try {
            dai::boundTreewidth(factor_graph, &dai::eliminationCost_MinFill, maxstates);
        } catch( dai::Exception &e ) {
            if( e.getCode() == dai::Exception::OUT_OF_MEMORY )
                cout << "error: cannot compute junction tree (need more than " << maxstates << " states)" << endl;
        }
#endif

        // junction tree
        dai::PropertySet opts;
        //opts = dai::PropertySet()("updates", string("HUGIN"))("verbose", (size_t)0);
        //inference_algorithm_ = new dai::JTree(factor_graph, opts);

        opts = dai::PropertySet()("updates", string("SEQRND"))("logdomain", false)("tol", 1e-3)("maxiter", (size_t)20)("maxtime", double(1))("damping", double(.2))("verbose", (size_t)0);
        inference_algorithm_ = new dai::BP(factor_graph, opts);

        //opts = dai::PropertySet()("updates", string("SEQRND"))("clamp", string("CLAMP_VAR"))("choose", string("CHOOSE_RANDOM"))("min_max_adj", double(10))("bbp_props", string(""))("bbp_cfn", string(""))("recursion", string("REC_FIXED"))("tol", 1e-3)("rec_tol", 1e-3)("maxiter", (size_t)100)("verbose", (size_t)0);
        //inference_algorithm_ = new dai::CBP(factor_graph, opts);

        //opts = dai::PropertySet()("updates", string("SEQRND"))("cavity", string("FULL"))("logdomain", false)("tol", 1e-3)("maxiter", (size_t)100)("maxtime", double(1))("damping", double(.2))("verbose", (size_t)0);
        //inference_algorithm_ = new dai::LC(factor_graph, opts);

        //opts = dai::PropertySet()("updates", string("LINEAR"))("inits", string("RESPPROP"))("logdomain", false)("tol", 1e-3)("maxiter", (size_t)100)("maxtime", double(1))("damping", double(.2))("verbose", (size_t)10);
        //inference_algorithm_ = new dai::MR(factor_graph, opts);

        //opts = dai::PropertySet()("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-3)("maxiter", (size_t)100)("maxtime", double(1));
        //inference_algorithm_ = new dai::HAK(factor_graph, opts);

        cout << "Properties: " << inference_algorithm_->printProperties() << endl;
        inference_algorithm_->init();
    }

    void destroy_inference_algorithm() {
        delete inference_algorithm_;
        inference_algorithm_ = 0;
    }

    void print_factor(ostream &os, int loc, const dai::Factor &factor, const string &name) const {
        os << "variables[loc=" << loc << "]=";
        for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it )
            os << " " << *it;
        os << endl << name << "[loc=" << loc << "]=" << endl;
        for( int j = 0; j < int(factor.nrStates()); ++j ) {
            os << "   ";
            map<dai::Var, size_t> states = dai::calcState(factor.vars(), j);
            for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it )
                os << " " << setw(2) << *it << "=" << states[*it];
            os << ":  j=" << setw(3) << j << ",  nbits=" << "na" << ",  value=" << factor[j] << endl;
        }
    }

    void print_factor_edbp(ostream &os, const dai::Factor &factor) const {
        os << factor.nrStates() << endl;
        for( int j = 0; j < int(factor.nrStates()); ++j )
            os << " " << factor[j];
        os << endl;
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
        need_to_recalculate_marginals_ = true;
    }

    // update factors for obtained obs for cell
    int update_factors(bool flag_action, int cell, int obs) {
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
            need_to_recalculate_marginals_ = true;
            return cell;
        } else {
            ++nflags_;
            return -1;
        }
    }

    void calculate_marginals(int factor_index = -1, bool print_marginals = false) const {
        if( need_to_recalculate_marginals_ ) {
            if( inference_type_ == 0 ) {
                apply_inference_libdai(factor_index);
                extract_marginals_from_inference_libdai(print_marginals);
            } else {
                apply_inference_edbp();
                extract_marginals_from_inference_edbp(print_marginals);
            }
            need_to_recalculate_marginals_ = false;
        }
    }

    void extract_marginals_from_inference_libdai(bool print_marginals = false) const {
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            marginals_[loc] = inference_algorithm_->belief(variables_[loc]);
            if( print_marginals )
                print_factor(cout, loc, marginals_[loc], "libdai_marginals_");
        }
    }

    void apply_inference_libdai(int factor_index) const {
        assert(inference_algorithm_ != 0);
        if( factor_index == -1 ) {
            for( int i = 0; i < int(factors_.size()); ++i )
                inference_algorithm_->fg().setFactor(i, factors_[i]);
            inference_algorithm_->init();
        } else {
            inference_algorithm_->fg().setFactor(factor_index, factors_[factor_index]);
            inference_algorithm_->init(factors_[factor_index].vars());
        }
        inference_algorithm_->run();
    }

    void extract_marginals_from_inference_edbp(bool print_marginals = false) const {
        ifstream ifs(edbp_tmp_fn_ + ".MAR");

        string buff;
        ifs >> buff;
        assert(buff == "MAR");
        int nlines = 0;
        ifs >> nlines;
        assert(nlines == 1);

        int nfactors = 0;
        ifs >> nfactors;
        assert(nfactors == nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            int nstates = 0;
            ifs >> nstates;
            assert(nstates == 2); // binary variables
            for( int j = 0; j < nstates; ++j ) {
                float p = 0;
                ifs >> p;
                marginals_[loc].set(j, p);
            }
            if( print_marginals ) print_factor(cout, loc, marginals_[loc], "edbp_marginals_");
        }
        ifs.close();
    }

    void apply_inference_edbp() const {
        // create model in given file
        ofstream ofs(edbp_tmp_fn_);
        ofs << "MARKOV"
            << endl
            << nrows_ * ncols_
            << endl;
        for( int loc = 0; loc < nrows_ * ncols_; ++loc )
            ofs << 2 << (loc < nrows_ * ncols_ - 1 ? " " : "");
        ofs << endl << nrows_ * ncols_ << endl;
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            const dai::Factor &factor = factors_[loc];
            ofs << factor.vars().size();
            for( dai::VarSet::const_reverse_iterator it = factor.vars().rbegin(); it != factor.vars().rend(); ++it ) {
                ofs << " " << it->label();
            }
            ofs << endl;
        }
        ofs << endl;
        for( int loc = 0; loc < int(factors_.size()); ++loc ) {
            print_factor_edbp(ofs, factors_[loc]);
            ofs << endl;
        }
        ofs.close();

        // call edbp solver
        string edbp_cmd = string("~/software/edbp/solver") + " " + edbp_tmp_fn_ + " " + edbp_evid_fn_ + " 0 MAR 2>/dev/null";
        system(edbp_cmd.c_str());
    }

    // compute full joint by multiplying factors (exponential)
    void compute_full_joint() const {
        full_joint_ = dai::Factor();
        for( int i = 0; i < nrows_ * ncols_; ++i )
            full_joint_ *= factors_[i];
        full_joint_.normalize();
    }

    // return the marginal distribution of the full joint over a cell variable
    pair<float, float> full_joint_marginal(int cell) const {
        dai::Factor marginal = full_joint_.marginal(variables_[cell]);
        return make_pair(float(marginal[0]), float(marginal[1]));
    }

#if 0
    void full_joint_marginal(int cell, dai::Factor &marginal) const {
        marginal = full_joint_.marginal(dai::VarSet(variables_[cell]));
    }

    // use junction tree algorithm to compute the exact marginals for each factor
    void apply_junction_tree() const {
        dai::PropertySet opts;
        factor_graph_ = dai::FactorGraph(factors_);
        //size_t maxstates = 1e8;
        //dai::boundTreewidth(factor_graph_, &dai::eliminationCost_MinFill, maxstates);
        jt_ = dai::JTree(factor_graph_, opts("updates", string("HUGIN")));
        jt_.init();
        jt_.run();
    }
    void apply_junction_tree(vector<float> &P) const {
        P.clear();
        apply_junction_tree();
        float mass = 0;
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            P.push_back(jt_.belief(factor_graph_.var(i))[0]);
            mass = P.back();
        }
        for( int i = 0; i < nrows_ * ncols_; ++i ) P[i] /= mass;
    }
    pair<float, float> jt_marginal(int cell) const {
        return make_pair(float(jt_.belief(factor_graph_.var(cell))[0]), float(jt_.belief(factor_graph_.var(cell))[1]));
    }
#endif

    // recommend action using information in the marginals
    int agent_get_action() const {
        assert(!need_to_recalculate_marginals_);
        float best_prob_for_open = 0, best_prob_for_flag = 0;
        vector<int> best_for_open, best_for_flag;
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            if( plays_.find(loc) != plays_.end() ) continue; // cell had been already played
            const dai::Factor &marginal = marginals_[loc];
            assert(marginal.nrStates() == 2);
            //pair<float, float> p((this->*marginal)(loc));    // get marginal on cell
            if( best_for_open.empty() || (marginal[0] >= best_prob_for_open) ) {
                if( best_for_open.empty() || (marginal[0] > best_prob_for_open) ) {
                    best_prob_for_open = marginal[0];
                    best_for_open.clear();
                }
                best_for_open.push_back(loc);
            }
            if( (nflags_ < nmines_) && (best_for_flag.empty() || (marginal[1] >= best_prob_for_flag)) ) {
                if( best_for_flag.empty() || (marginal[1] > best_prob_for_flag) ) {
                    best_prob_for_flag = marginal[1];
                    best_for_flag.clear();
                }
                best_for_flag.push_back(loc);
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
    cout << "SEED=" << seed << endl;
    unsigned short seeds[3];
    seeds[0] = seeds[1] = seeds[2] = seed;
    seed48(seeds);

    // create distributions
    ms_pbt_t pbt(nrows, ncols, nmines, 0, false);
    //ms_pbt_t pbt(nrows, ncols, nmines, true);

    // run for the specified number of trials
    for( int trial = 0; trial < ntrials; ) {
        minefield_t minefield(nrows, ncols, nmines);
        agent_initialize(nrows, ncols, nmines);

        // inference and print marginals
        pbt.reset();
        pbt.calculate_marginals(-1, false);

        bool win = true;
        vector<pair<int,int> > execution(nrows * ncols);
        for( int play = 0; play < nrows * ncols; ++play ) {
            int action = pbt.agent_get_action();

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

            // update agent's belief
            agent_update_state(agent_is_flag_action(action), agent_get_cell(action), obs);
            int factor_index = pbt.update_factors(pbt.is_flag_action(action), agent_get_cell(action), obs);
            pbt.calculate_marginals(factor_index, false);
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

