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

#define LIBDAI
#ifdef LIBDAI
#include <dai/alldai.h>
#endif

using namespace std;

#if 0
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
        vector<int> available_cells(ncells_, 0);
        set<int> forbidden;
        // do not place a mine at the initial cell or surrounding cells
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

        // place random mines in other cells
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
        //print(cout);
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

    void print(ostream &os) const {
        for( int c = 0; c < ncols_; ++c ) {
            for( int r = 0; r < nrows_; ++r ) {
                int cell = r * ncols_ + c;
                if( cells_[cell].mine_ )
                    os << " *";
                else
                    os << " " << cells_[cell].nmines_;
            }
            //os << endl;
        }
    }
};
#endif

pair<float, bool> KL_divergence(const vector<float> &P, const vector<float> &Q) {
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

pair<float, bool> JS_divergence(const vector<float> &P, const vector<float> &Q) {
    assert(P.size() == Q.size());
    vector<float> M(P.size(), 0);
    for( size_t i = 0; i < M.size(); ++i )
        M[i] = (P[i] + Q[i]) / 2;
    pair<float, bool> kl1 = KL_divergence(P, M);
    pair<float, bool> kl2 = KL_divergence(Q, M);
    assert(kl1.second && kl2.second);
    return make_pair((kl1.first + kl2.first) / 2, true);
}

#ifdef LIBDAI
struct pocman_cbt_t {
    int nrows_;
    int ncols_;
    int nghosts_;
    bool noisy_;

    int *maze_;

    struct level_t {
    };

    struct state_t {
    };

    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;

    mutable dai::Factor joint_;
    mutable dai::FactorGraph jt_fg_;
    mutable dai::JTree jt_;
    mutable dai::FactorGraph approx_inference_fg_;
    mutable dai::InfAlg *approx_inference_algorithm_;

    pocman_cbt_t(int nrows, int ncols, int nghosts, bool noisy = false)
      : nrows_(nrows), ncols_(ncols), nghosts_(nghosts), noisy_(noisy) {
        variables_ = vector<dai::Var>(nrows_ * ncols_);
        for( int i = 0; i < nrows_ * ncols_; ++i )
            variables_[i] = dai::Var(i, 2);
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
        cout << "cbt: factor graph:"
             << " #variables=" << variables_.size()
             << ", #factors=" << factors_.size()
             << endl;
    }

    void reset() {
#if 0
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            dai::Factor &factor = factors_[i];
            float p = 1.0 / (1 << factor.vars().size());
            for( int j = 0; j < (1 << factor.vars().size()); ++j )
                factor.set(j, p);
        }
        plays_.clear();
        nflags_ = 0;
        approx_inference_algorithm_ = 0;
#endif
    }

    void update(int action, int obs) {
#if 0
        assert(plays_.find(cell) == plays_.end());
        plays_.insert(cell);
        int row = cell / ncols_, col = cell % ncols_;
        cout << "cbt: flag=" << flag_action
             << ", cell=" << cell << ":(" << col << "," << row << ")"
             << ", obs=" << obs << endl;
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
#endif
    }

    void apply_junction_tree() const {
#if 0
        dai::PropertySet opts;
        jt_fg_ = dai::FactorGraph(factors_);
        //size_t maxstates = 1e8;
        //dai::boundTreewidth(jt_fg_, &dai::eliminationCost_MinFill, maxstates);
        jt_ = dai::JTree(jt_fg_, opts("updates", string("HUGIN")));
        jt_.init();
        jt_.run();
#endif
    }
    void apply_junction_tree(vector<float> &P) const {
#if 0
        P.clear();
        apply_junction_tree();
        float mass = 0;
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            P.push_back(jt_.belief(jt_fg_.var(i))[0]);
            mass = P.back();
        }
        for( int i = 0; i < nrows_ * ncols_; ++i ) P[i] /= mass;
#endif
    }
    pair<float, float> jt_marginal(int cell) const {
#if 0
        return make_pair(float(jt_.belief(jt_fg_.var(cell))[0]), float(jt_.belief(jt_fg_.var(cell))[1]));
#endif
    }

    void apply_approx_inference() const {
#if 0
        dai::PropertySet opts;
        approx_inference_fg_ = dai::FactorGraph(factors_);
        delete approx_inference_algorithm_;
        approx_inference_algorithm_ = new dai::BP(approx_inference_fg_, opts("updates", string("SEQRND"))("logdomain", true)("tol", 1e-9)("maxiter", (size_t)10000));
        //approx_inference_algorithm_ = new dai::Gibbs(approx_inference_fg_, opts("maxiter", (size_t)10000)("burnin", (size_t)100)("restart", (size_t)10000));
        //approx_inference_algorithm_ = new dai::HAK(approx_inference_fg_, opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));
        approx_inference_algorithm_->init();
        approx_inference_algorithm_->run();
#endif
    }
    void apply_approx_inference(vector<float> &P) const {
#if 0
        P.clear();
        apply_approx_inference();
        float mass = 0;
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            P.push_back(approx_inference_algorithm_->belief(approx_inference_fg_.var(i))[0]);
            mass = P.back();
        }
        for( int i = 0; i < nrows_ * ncols_; ++i ) P[i] /= mass;
#endif
    }
    pair<float, float> approx_inference_marginal(int cell) const {
#if 0
        return make_pair(float(approx_inference_algorithm_->belief(approx_inference_fg_.var(cell))[0]),
                         float(approx_inference_algorithm_->belief(approx_inference_fg_.var(cell))[1]));
#endif
    }

    int agent_get_action(pair<float, float> (ms_cbt_t::*marginal)(int) const) const {
#if 0
        float best_prob_for_open = 0, best_prob_for_flag = 0;
        vector<int> best_for_open, best_for_flag;
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            if( plays_.find(i) != plays_.end() ) continue;
            pair<float, float> p((this->*marginal)(i));
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
        if( !best_for_open.empty() && (best_prob_for_open >= best_prob_for_flag) ) {
            random_shuffle(best_for_open.begin(), best_for_open.end());
            return best_for_open.back() + (nrows_ * ncols_);
        } else if( !best_for_flag.empty() && (best_prob_for_open < best_prob_for_flag) ) {
            random_shuffle(best_for_flag.begin(), best_for_flag.end());
            return best_for_flag.back();
        }
        else if( !best_for_open.empty() ) {
            random_shuffle(best_for_open.begin(), best_for_open.end());
            return best_for_open.back() + (nrows_ * ncols_);
        } else if( nflags_ < nmines_ ) {
            assert(!best_for_flag.empty());
            random_shuffle(best_for_flag.begin(), best_for_flag.end());
            return best_for_flag.back();
        } else {
            assert(0);
            return 0;
        }
#endif
    }

    void print_marginals(ostream &os, pair<float, float> (ms_cbt_t::*marginal)(int) const, int prec = 2) {
#if 0
        int old_prec = os.precision();
        os << setprecision(prec);
        for( int row = 0; row < nrows_; ++row ) {
            for( int col = 0; col < ncols_; ++col ) {
                int cell = row * ncols_ + col;
                //float p = joint_marginal(cell);
                pair<float, float> p = (this->*marginal)(cell);
                os << "cbt: P[ (" << col << "," << row << ")=0 ] = " << p.first
                   << (p.second == 0.0 ? "*" : (p.first == 0.0 ? "-" : ""))
                   << endl;
            }
        }
        os << setprecision(old_prec);
#endif
    }
};
#endif

void usage(ostream &os) {
    os << endl
       << "Usage: mines [{-t | --ntrials} <ntrials>]" << endl
       << "             [{-r | --nrows} <nrows>]" << endl
       << "             [{-c | --ncols} <ncols>]" << endl
       << "             [{-m | --nmines} <nmines>]" << endl
       << "             [{-s | --seed} <seed>]" << endl
       << "             [{-v | --verbose}]" << endl
       << "             [{-p | --policy} <policy>]" << endl
       << "             [{-w | --width} <width>]" << endl
       << "             [{-d | --depth} <depth>]" << endl
       << "             [{-P | --print-deterministic-execution} <n>]" << endl
       << "             [{-? | --help}]" << endl
       << endl
       << "where <ntrials> is a non-negative integer telling the number of games to" << endl
       << "play (default is 1), <nrows> and <ncols> are positive integers telling" << endl
       << "the dimensions of the minefield (default is 16x16), <nmines> is a positive" << endl
       << "integer telling the number of hidden mines in the minefield (default is 40)," << endl
       << "<seed> is an integer for setting the seed of the random number generator" << endl
       << "(default is 0), <policy> is a string describing the policy to use (default" << endl
       << "is \"base-policy:direct\"), and <width> and <depth> are parameters for the" << endl
       << "policy (the default policy is parameter-free)." << endl
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
    bool print_deterministic_executions = false;
    int executions_to_print = 0;

    string policy = "base-policy:direct";
    int width = 100;
    int depth = 10;

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
        } else if( !strcmp(argv[0], "-p") || !strcmp(argv[0], "--policy") ) {
            policy = argv[1];
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-w") || !strcmp(argv[0], "--width") ) {
            width = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-d") || !strcmp(argv[0], "--depth") ) {
            depth = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-P") || !strcmp(argv[0], "--print-deterministic-executions") ) {
            print_deterministic_executions = true;
            executions_to_print = atoi(argv[1]);
            argc -= 2;
            argv += 2;
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

#ifdef LIBDAI
    // create distributions
    ms_cbt_t cbt(nrows, ncols, nmines, false);
    //ms_cbt_t cbt(nrows, ncols, nmines, true);
#endif

    // run for the specified number of trials
    if( print_deterministic_executions ) ntrials = executions_to_print;
    for( int trial = 0; trial < ntrials; ) {
        minefield_t minefield(nrows, ncols, nmines);
        agent_initialize(nrows, ncols, nmines);
#ifdef LIBDAI
        cbt.reset();
        //cbt.apply_junction_tree();
        cbt.apply_approx_inference();
#endif

        bool win = true;
        int previous_nguesses = agent_get_nguesses();
        vector<pair<int,int> > execution(nrows * ncols);
        for( int play = 0; play < nrows * ncols; ++play ) {
            int is_guess = 2;
#ifndef LIBDAI
            int action = agent_get_action(&is_guess);
#else
            //int action = cbt.agent_get_action(&ms_cbt_t::jt_marginal);
            int action = cbt.agent_get_action(&ms_cbt_t::approx_inference_marginal);
#endif
            if( play == 0 ) {
                assert(!agent_is_flag_action(action));
                int cell = agent_get_cell(action);
                minefield.sample(cell);
                assert(!minefield.cells_[cell].mine_);
            }
            int obs = minefield.apply_action(action, verbose);
            if( obs == 9 ) {
                win = false;
                break;
            }
            agent_update_state(agent_is_flag_action(action), agent_get_cell(action), obs);
            execution[play] = make_pair(action, obs);

#ifdef LIBDAI
            cout << "ABOUT TO UPDATE:"
                 << " play=" << play
                 << ", action=" << action
                 << ", obs=" << obs
                 << ", is-guess=" << is_guess
                 << endl;

#if 0
            if( (play > 0) && !agent_is_flag_action(action) && (cbt.joint_marginal(agent_get_cell(action)) != 1.0) ) {
                cout << "cbt: INSECURE ACTION: open cell ("
                     << agent_get_cell(action) / ncols << ","
                     << agent_get_cell(action) % ncols << ")"
                     << endl;
            }
#endif

            // update factors
            cbt.update(agent_is_flag_action(action), agent_get_cell(action), obs);

#if 0
            // compute and print exact marginals via junction tree
            vector<float> P;
            cout << "Marginals (exact: junction tree):" << endl;
            cbt.apply_junction_tree(P);
            cbt.print_marginals(cout, &ms_cbt_t::jt_marginal);
#endif

#if 1
            // approximate and print marginals
            vector<float> Q;
            cout << "Marginals (approx. inference):" << endl;
            cbt.apply_approx_inference(Q);
            cbt.print_marginals(cout, &ms_cbt_t::approx_inference_marginal);
#endif

#if 0
            // calculate KL-divergence
            pair<float, bool> kl1 = KL_divergence(P, Q);
            pair<float, bool> kl2 = KL_divergence(Q, P);
            pair<float, bool> js = js_divergence(P, Q);
            cout << "KL-divergence: v1=" << kl1.first << ", s1=" << kl1.second << ", v2=" << kl2.first << ", s2=" << kl2.second << ", R=" << (kl1.first * kl2.first) / (kl1.first + kl2.first) << ", J=" << js.first << endl;
#endif
#endif
        }
        if( win ) {
            agent_declare_win(!print_deterministic_executions);
            if( print_deterministic_executions && (agent_get_nguesses() == 1 + previous_nguesses) ) {
                ++trial;
                cout << "minefield:";
                minefield.print(cout);
                cout << endl << "execution: ";
                for( int i = 0; i < nrows * ncols; ++i ) {
                    int action = execution[i].first, obs = execution[i].second;
                    int cell = agent_get_cell(action), row = cell / ncols, col = cell % ncols;
                    cout << (agent_is_flag_action(action) ?  "flag " : "open ")
                         << row << " " << col << " obs " << obs << " ";
                }
                cout << endl;
            }
        } else {
            agent_declare_lose(!print_deterministic_executions);
        }
        if( !print_deterministic_executions ) ++trial;
    }
    agent_finalize();
    return 0;
}

