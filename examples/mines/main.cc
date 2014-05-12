/*
 *  Copyright (C) 2013 Universidad Simon Bolivar
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
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <stdlib.h>
#include <math.h>

#include "full_joint_tracking.h"
#include "causal_belief_tracking.h"

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

struct ms_bt_t {
    int nrows_;
    int ncols_;

    ms_bt_t(int nrows, int ncols) : nrows_(nrows), ncols_(ncols) { }
    virtual ~ms_bt_t() { }

    virtual void reset() = 0;
    virtual float marginal(int cell) const = 0;
    virtual void update(bool flag_action, int cell, int obs) = 0;
    virtual void print_marginals(ostream &os, int prec = 2) = 0;

    bool compare_marginals(const ms_bt_t &bt, float epsilon) const {
        for( int c = 0; c < ncols_; ++c ) {
            for( int r = 0; r < nrows_; ++r ) {
                int cell = r * ncols_ + c;
                if( fabs(marginal(cell) - bt.marginal(cell)) > epsilon )
                    return false;
            }
        }
        return true;
    }
};

struct ms_fjt_t : public ms_bt_t, public ProbabilisticBeliefTracking::full_joint_tracking_t<float> {
    ms_fjt_t(int nrows, int ncols)
      : ms_bt_t(nrows, ncols),
        ProbabilisticBeliefTracking::full_joint_tracking_t<float>(nrows * ncols, 2) {
        set_uniform_distribution();
    }
    virtual ~ms_fjt_t() { }

    int sum_of_mines_around_cell(int cell, const BeliefTracking::valuation_t &valuation) {
        int r = cell / ncols_, c = cell % ncols_;
        int sum = 0;
        for( int dr = -1; dr < 2; ++dr ) {
            if( (r + dr < 0) || (r + dr >= nrows_) ) continue;
            for( int dc = -1; dc < 2; ++dc ) {
                if( (c + dc < 0) || (c + dc >= ncols_) ) continue;
                if( true || (dr != 0) || (dc != 0) ) sum += valuation[(r + dr) * ncols_ + (c + dc)];
            }
        }
        return sum;
    }

    void progress(int action, BeliefTracking::valuation_t &valuation) {
    }

    float filter(int obs, int cell, const BeliefTracking::valuation_t &valuation) {
        return sum_of_mines_around_cell(cell, valuation) != obs ? 0 : 1;
    }

    virtual void reset() {
        set_uniform_distribution();
    }

    virtual float marginal(int cell) const {
        BeliefTracking::event_t event;
        event.push_back(make_pair(cell, 1));
        return probability(event);
    }

    virtual void update(bool flag_action, int cell, int obs) {
        int row = cell / ncols_, col = cell % ncols_;
        cout << "fjt: flag=" << flag_action << ", cell=" << cell << ":(" << col << "," << row << "), obs=" << obs << ", #non-zero-entries=" << non_zero_entries() << endl;
        if( !flag_action ) {
            BeliefTracking::belief_tracking_t::det_progress_func_t progress = 0;
            BeliefTracking::belief_tracking_t::filter_func_t filter = 0;
            progress = static_cast<BeliefTracking::belief_tracking_t::det_progress_func_t>(&ms_fjt_t::progress);
            filter = static_cast<BeliefTracking::belief_tracking_t::filter_func_t>(&ms_fjt_t::filter);
            progress_and_filter(cell, obs, progress, filter);
        } else {
            for( int k = 0; k < nvaluations(); ++k ) {
                if( xprobability(k) == 0 ) continue;
                decode_index(k);
                assert(valuation_[cell] == 1);
            }
        }
    }

    virtual void print_marginals(ostream &os, int prec = 2) {
        int old_prec = os.precision();
        os << setprecision(prec);
        for( int c = 0; c < ncols_; ++c ) {
            for( int r = 0; r < nrows_; ++r ) {
                int cell = r*ncols_ + c;
                float p = marginal(cell);
                os << "fjt: mar(" << c << "," << r << ")=" << p << endl;
            }
        }
        os << setprecision(old_prec);
    }
};

struct ms_cbt_t : public ms_bt_t, public ProbabilisticBeliefTracking::causal_belief_tracking_t<float> {
    ms_cbt_t(int nrows, int ncols, const ms_fjt_t *fjt)
      : ms_bt_t(nrows, ncols),
        ProbabilisticBeliefTracking::causal_belief_tracking_t<float>(nrows * ncols, fjt) {
        for( int c = 0; c < ncols_; ++c ) {
            for( int r = 0; r < nrows_; ++r ) {
                int beam_index = r * ncols_ + c, nvars = 0;

                // set string for beam
                stringstream ss;
                ss << beam_index << ":cell(" << c << "," << r << ")";
                beams_[beam_index].set_string(ss.str());

                // calculate number of variables in beam
                if( (c == 0) || (c == ncols_ - 1) )
                    nvars = (r == 0) || (r == nrows_ - 1) ? 4 : 6;
                else
                    nvars = (r == 0) || (r == nrows_ - 1) ? 6 : 9;
                beams_[beam_index].allocate(nvars, 2);

                // calculate inverse map for beam
                std::vector<int> inverse_variable_map;
                inverse_variable_map.reserve(nvars);
                for( int dc = -1; dc < 2; ++dc ) {
                    if( (c + dc < 0) || (c + dc >= ncols_) ) continue;
                    for( int dr = -1; dr < 2; ++dr ) {
                        if( (r + dr < 0) || (r + dr >= nrows_) ) continue;
                        int nc = c + dc, nr = r + dr;
                        inverse_variable_map.push_back(nr * ncols_ + nc);
                    }
                }
                beams_[beam_index].set_variables(inverse_variable_map);
            }
        }
        calculate_variables();
        reset();
    }
    virtual ~ms_cbt_t() { }

    int sum_of_mines_around_cell(int cell, const BeliefTracking::valuation_t &valuation) {
        int r = cell / ncols_, c = cell % ncols_;
        int sum = 0;
        for( int dr = -1; dr < 2; ++dr ) {
            if( (r + dr < 0) || (r + dr >= nrows_) ) continue;
            for( int dc = -1; dc < 2; ++dc ) {
                if( (c + dc < 0) || (c + dc >= ncols_) ) continue;
                if( true || (dr != 0) || (dc != 0) ) sum += valuation[(r + dr) * ncols_ + (c + dc)];
            }
        }
        return sum;
    }

    void progress(int action, BeliefTracking::valuation_t &valuation) {
    }

    float filter(int obs, int cell, const BeliefTracking::valuation_t &valuation) {
        return sum_of_mines_around_cell(cell, valuation) != obs ? 0 : 1;
    }

    virtual void reset() {
        set_uniform_distribution();
    }

    virtual float marginal(int cell) const {
        BeliefTracking::event_t event;
        event.push_back(make_pair(cell, 1));
        return probability(event);
    }

    virtual void update(bool flag_action, int cell, int obs) {
        int row = cell / ncols_, col = cell % ncols_;
        cout << "cbt: flag=" << flag_action << ", cell=" << cell << ":(" << col << "," << row << "), obs=" << obs << endl;
        if( !flag_action ) {
            BeliefTracking::belief_tracking_t::det_progress_func_t progress = 0;
            BeliefTracking::belief_tracking_t::filter_func_t filter = 0;
            progress = static_cast<BeliefTracking::belief_tracking_t::det_progress_func_t>(&ms_cbt_t::progress);
            filter = static_cast<BeliefTracking::belief_tracking_t::filter_func_t>(&ms_cbt_t::filter);
            cout << "BLAI=" << verify_join2(*joint_, 0.0001) << endl;
            progress_and_filter(cell, obs, progress, filter);
        }
    }

    virtual void print_marginals(ostream &os, int prec = 2) {
        int old_prec = os.precision();
        os << setprecision(prec);
        for( int c = 0; c < ncols_; ++c ) {
            for( int r = 0; r < nrows_; ++r ) {
                int cell = r*ncols_ + c;
                float p = marginal(cell);
                os << "cbt: mar(" << c << "," << r << ")=" << p << endl;
            }
        }
        os << setprecision(old_prec);
    }

    bool verify_join(const ms_fjt_t &fjt, float epsilon) const {
        for( ProbabilisticBeliefTracking::causal_belief_tracking_t<float>::const_join_iterator_type2 it = begin_join(&fjt); it != end_join(); ++it ) {
            int index = it.get_index();
            float p = it.get_probability();
            if( fabs(p - fjt.xprobability(index)) > epsilon ) {
                cout << "join: val=" << it.get_valuation() << ", index=" << index << ", cbt=" << p << ", fjt=" << fjt.xprobability(index) << endl;
                return false;
            }
        }
        return true;
    }

    bool verify_join2(const ProbabilisticBeliefTracking::joint_distribution_t<float> &fjt, float epsilon) const {
        cout << "vj2: joint=" << &fjt << endl;
        for( ProbabilisticBeliefTracking::causal_belief_tracking_t<float>::const_join_iterator_type2 it = begin_join(&fjt); it != end_join(); ++it ) {
            int index = it.get_index();
            float p = it.get_probability();
            if( fabs(p - fjt.xprobability(index)) > epsilon ) {
                cout << "join: val=" << it.get_valuation() << ", index=" << index << ", cbt=" << p << ", fjt=" << fjt.xprobability(index) << endl;
                return false;
            }
            if( fabs(p - fjt.probability(it.get_valuation())) > epsilon ) {
                cout << "ZZZZZZZZ" << endl;
            }
        }
        return true;
    }

    bool verify_beams(const ms_fjt_t &fjt, float epsilon) const {
        for( int k = 0; k < nbeams(); ++k ) {
            const ProbabilisticBeliefTracking::beam_t<float> &beam = beams_[k];
            for( int index = 0; index < beam.nvaluations(); ++index ) {
                BeliefTracking::valuation_t valuation(beam.nvars(), 0);
                beam.decode_index(index, valuation);
                BeliefTracking::event_t event;
                for( int i = 0; i < beam.nvars(); ++i )
                    event.push_back(make_pair(beam.inverse_map_variable(i), valuation[i]));
                if( fabs(beam.xprobability(index) - fjt.probability(event)) > epsilon ) {
                    cout << "beam=" << beam.beam_string()
                         << ", event=" << event
                         << ", cbt=" << beam.xprobability(index)
                         << ", fjt=" << fjt.probability(event)
                         << endl;
                    beam.print(cout);
                    return false;
                }
            }
        }
        return true;
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

    // create distributions
    ms_fjt_t fjt(nrows, ncols);
    ms_cbt_t cbt(nrows, ncols, &fjt);

    // run for the specified number of trials
    if( print_deterministic_executions ) ntrials = executions_to_print;
    for( int trial = 0; trial < ntrials; ) {
        minefield_t minefield(nrows, ncols, nmines);
        agent_initialize(nrows, ncols, nmines);
        fjt.reset();
        cbt.reset();

        bool win = true;
        int previous_nguesses = agent_get_nguesses();
        vector<pair<int,int> > execution(nrows * ncols);
        for( int play = 0; play < nrows * ncols; ++play ) {
            int action = agent_get_action();
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

            cout << "verify join1=" << flush << cbt.verify_join(fjt, 0.0001) << endl;
            cout << "verify beams=" << flush << cbt.verify_beams(fjt, 0.0001) << endl;
            cout << "verify join2=" << flush << cbt.verify_join2(*cbt.joint_, 0.0001) << endl;
            cout << "verify join3" << endl; cbt.my_verify_join(&fjt);

            cout << "ABOUT TO UPDATE" << endl;
            cbt.update(agent_is_flag_action(action), agent_get_cell(action), obs);
            cbt.print_marginals(cout);
            fjt.update(agent_is_flag_action(action), agent_get_cell(action), obs);
            fjt.print_marginals(cout);
            cout << "DONE W/  UPDATE" << endl;
            bool comp = fjt.compare_marginals(cbt, 0.0001);
            if( false && !comp ) {
                fjt.print_marginals(cout);
                cbt.print_marginals(cout);
            }
            //cbt.print(cout);
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
    return 0;
}

