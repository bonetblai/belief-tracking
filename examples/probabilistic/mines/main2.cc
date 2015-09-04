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

struct ms_cbt_t {
    int nrows_;
    int ncols_;
    int nmines_;
    bool noisy_;

    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;
    vector<int> centers_;

    mutable dai::Factor joint_;
    mutable dai::FactorGraph jt_fg_;
    mutable dai::JTree jt_;
    mutable dai::FactorGraph approx_inference_fg_;
    mutable dai::InfAlg *approx_inference_algorithm_;

    set<int> plays_;
    int nflags_;

    ms_cbt_t(int nrows, int ncols, int nmines, bool noisy = false) : nrows_(nrows), ncols_(ncols), nmines_(nmines), noisy_(noisy) {
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

    void update(bool flag_action, int cell, int obs) {
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
    }

    void compute_joint() const {
        joint_ = dai::Factor();
        for( int i = 0; i < nrows_ * ncols_; ++i )
            joint_ *= factors_[i];
        joint_.normalize();
    }
    pair<float, float> joint_marginal(int cell) const {
        return make_pair(float(joint_.marginal(dai::VarSet(variables_[cell]))[0]), float(joint_.marginal(dai::VarSet(variables_[cell]))[1]));
    }

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
        approx_inference_algorithm_ = new dai::BP(approx_inference_fg_, opts("updates", string("SEQRND"))("logdomain", true)("tol", 1e-9)("maxiter", (size_t)10000));
        //approx_inference_algorithm_ = new dai::Gibbs(approx_inference_fg_, opts("maxiter", (size_t)10000)("burnin", (size_t)100)("restart", (size_t)10000));
        //approx_inference_algorithm_ = new dai::HAK(approx_inference_fg_, opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(1)));
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

    int agent_get_action(pair<float, float> (ms_cbt_t::*marginal)(int) const) const {
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
    }
    int agent_is_flag_action(int action) {
        return action < nrows_ * ncols_ ? 1 : 0;
    }
    int agent_get_cell(int action) {
        bool flag = action < nrows_ * ncols_;
        return flag ? action : action - (nrows_ * ncols_);
    }

    void print_marginals(ostream &os, pair<float, float> (ms_cbt_t::*marginal)(int) const, int prec = 2) {
        int old_prec = os.precision();
        os << setprecision(prec);
        for( int row = 0; row < nrows_; ++row ) {
            os << "|";
            for( int col = 0; col < ncols_; ++col ) {
                int cell = row * ncols_ + col;
                //float p = joint_marginal(cell);
                pair<float, float> p = (this->*marginal)(cell);

                os << " " << setw(4) << 1 - p.first << (plays_.find(cell) != plays_.end() ? "*" : " ") << "|";
#if 0
                os << "cbt: P[ (" << col << "," << row << ")=0 ] = " << p.first
                   << (p.second == 0.0 ? "*" : (p.first == 0.0 ? "-" : ""))
                   << endl;
#endif
            }
            os << endl;
        }
        os << setprecision(old_prec);
    }
};

#if 0
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
    ms_cbt_t(int nrows, int ncols)
      : ms_bt_t(nrows, ncols),
        ProbabilisticBeliefTracking::causal_belief_tracking_t<float>(nrows * ncols) {
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

    void verify_join(const ms_fjt_t *fjt, float epsilon) const {
        for( ProbabilisticBeliefTracking::causal_belief_tracking_t<float>::const_join_iterator_type2 it = begin_join(); it != end_join(); ++it ) {
            int index = it.get_index();
            float p = it.get_probability();
            if( fabs(p - fjt->xprobability(index)) > epsilon ) {
                cout << "join: val=" << it.get_valuation()
                     << ", index=" << index
                     << ", cbt=" << p
                     << ", fjt=" << fjt->xprobability(index)
                     << endl;
            }
        }
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
    ms_cbt_t cbt(nrows, ncols, nmines, false);
    //ms_cbt_t cbt(nrows, ncols, nmines, true);

    // run for the specified number of trials
    for( int trial = 0; trial < ntrials; ) {
        minefield_t minefield(nrows, ncols, nmines);
        agent_initialize(nrows, ncols, nmines);

        cbt.reset();
        //cbt.apply_junction_tree();
        cbt.apply_approx_inference();

        bool win = true;
        vector<pair<int,int> > execution(nrows * ncols);
        for( int play = 0; play < nrows * ncols; ++play ) {
            int is_guess = 2;
            //int action = agent_get_action(&is_guess);
            //int action = cbt.agent_get_action(&ms_cbt_t::jt_marginal);
            int action = cbt.agent_get_action(&ms_cbt_t::approx_inference_marginal);
            if( play == 0 ) {
                assert(!agent_is_flag_action(action));
                int cell = agent_get_cell(action);
                minefield.sample(cell);
                assert(!minefield.cells_[cell].mine_);
            }
            int obs = minefield.apply_action(action, verbose);
            if( obs == 9 ) {
                cout << "BOOM" << endl;
                win = false;
                break;
            }
            agent_update_state(agent_is_flag_action(action), agent_get_cell(action), obs);
            execution[play] = make_pair(action, obs);

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
            pair<float, bool> kl1 = kl_divergence(P, Q);
            pair<float, bool> kl2 = kl_divergence(Q, P);
            pair<float, bool> js = js_divergence(P, Q);
            cout << "KL-divergence: v1=" << kl1.first << ", s1=" << kl1.second << ", v2=" << kl2.first << ", s2=" << kl2.second << ", R=" << (kl1.first * kl2.first) / (kl1.first + kl2.first) << ", J=" << js.first << endl;
#endif

#if 0
            cout << "VERIFY JOIN-1" << endl;
            cbt.verify_join(&fjt, 0.0001);
            //cout << "verify beams=" << flush << cbt.verify_beams(fjt, 0.0001) << endl;
            cout << "VERIFY JOIN-2" << endl;
            static_cast<ProbabilisticBeliefTracking::causal_belief_tracking_t<float>*>(&cbt)->verify_join(&fjt);

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

