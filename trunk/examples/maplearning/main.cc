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

using namespace std;

// Forward references
struct cellmap_t;
struct tracking_t;

struct marginal_t : public vector<float> {
    marginal_t(size_t size = 0, float p = 0) : vector<float>(size, p) { }

    const marginal_t& operator*=(float p) {
        for( size_t i = 0; i < size(); ++i )
            (*this)[i] *= p;
        return *this;
    }
    const marginal_t& operator/=(float p) {
        return *this *= (1 / p);
    }
    const marginal_t& operator+=(const marginal_t &m) {
        assert(size() == m.size());
        for( size_t i = 0; i < size(); ++i )
            (*this)[i] += m[i];
        return *this;
    }
    const marginal_t& operator-=(const marginal_t &m) {
        assert(size() == m.size());
        for( size_t i = 0; i < size(); ++i )
            (*this)[i] -= m[i];
        return *this;
    }

    void print(ostream &os) const {
        //os << "[";
        for( size_t i = 0; i < size(); ++i )
            //os << " " << i << "->" << (*this)[i];
            os << " " << (*this)[i] << (i + 1 < size() ? "," : "");
        //os << " ]";
    }
};

inline ostream& operator<<(ostream &os, const marginal_t &marginal) {
    marginal.print(os);
    return os;
}

pair<float, bool> KL_divergence(const marginal_t &P, const marginal_t &Q) {
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

pair<float, bool> JS_divergence(const marginal_t &P, const marginal_t &Q) {
    assert(P.size() == Q.size());
    marginal_t M(P.size(), 0);
    for( size_t i = 0; i < M.size(); ++i )
        M[i] = (P[i] + Q[i]) / 2;
    pair<float, bool> kl1 = KL_divergence(P, M);
    pair<float, bool> kl2 = KL_divergence(Q, M);
    assert(kl1.second && kl2.second);
    return make_pair((kl1.first + kl2.first) / 2, true);
}


// General Tracking Algorithm
struct tracking_t {
    string name_;
    const cellmap_t &cellmap_;
    int nloc_;
    int nlabels_;

    vector<vector<marginal_t> > marginals_;

    tracking_t(const string &name, const cellmap_t &cellmap);
    virtual ~tracking_t() { }
    virtual void initialize(int initial_loc) = 0;
    virtual void update(int last_action, int obs) = 0;
    virtual void calculate_marginals() = 0;
    virtual void get_marginal(int var, marginal_t &marginal) const = 0;

    void store_marginals() {
        marginals_.push_back(vector<marginal_t>(nloc_ + 1));
        for( int var = 0; var < nloc_; ++var )
            get_marginal(var, marginals_.back()[var]);
        get_marginal(nloc_, marginals_.back()[nloc_]);
    }

    const marginal_t& stored_marginal(int t, int var) const {
        assert(t < int(marginals_.size()));
        const vector<marginal_t> &marginals_at_time_t = marginals_[t];
        assert(var < int(marginals_at_time_t.size()));
        return marginals_at_time_t[var];
    }

    int MAP_on_var(int var) const {
        marginal_t marginal;
        get_marginal(var, marginal);
        float max_probability = 0;
        int best = 0;
        for( size_t i = 0; i < marginal.size(); ++i ) {
            if( marginal[i] >= max_probability ) {
                max_probability = marginal[i];
                best = i;
            }
        }
        return best;
    }
};


// General Action Selection Policy
struct action_selection_t {
    const cellmap_t &cellmap_;
    action_selection_t(const cellmap_t &cellmap) : cellmap_(cellmap) { }
    virtual ~action_selection_t() { }
    virtual int select_action(const tracking_t *tracking = 0) const = 0;
};


struct coord_t {
    int col_;
    int row_;
    static int ncols_;
    coord_t(int col, int row) : col_(col), row_(row) { }
    coord_t(int loc) : col_(loc % ncols_), row_(loc / ncols_) { }
    int as_index() const { return row_ * ncols_ + col_; }
    void print(ostream &os) const {
        os << "(" << col_ << "," << row_ << ")";
    }
};

int coord_t::ncols_ = 0;

inline ostream& operator<<(ostream &os, const coord_t &coord) {
    coord.print(os);
    return os;
}

struct cell_t {
    int label_;
    cell_t(int label = 0) : label_(label) { }
};

struct cellmap_t {
    int nrows_;
    int ncols_;
    int nlabels_;

    float pa_;
    float po_;
    float pc_;

    vector<cell_t> cells_;

    cellmap_t(int nrows, int ncols, int nlabels, float pa, float po, float pc)
      : nrows_(nrows), ncols_(ncols), nlabels_(nlabels), pa_(pa), po_(po), pc_(pc) {
        cells_ = vector<cell_t>(nrows_ * ncols_);
    }

    void set_labels(const int *labels, size_t size) {
        assert(size == cells_.size());
        for( size_t i = 0; i < size; ++i )
            cells_[i].label_ = labels[i];
    }
    void set_labels(const vector<int> labels) {
        set_labels(&labels[0], labels.size());
    }

    void sample_labels() {
        vector<int> labels(cells_.size(), 0);
        for( size_t i = 0; i < labels.size(); ++i )
            labels[i] = lrand48() % nlabels_;
        set_labels(labels);
    }

    // action labels
    enum { up, right, down, left };

    // computes P(new_loc | old_lod, action)
    float loc_probability(int action, int old_loc, int new_loc) const {
        float p = 0;
        coord_t coord(old_loc), new_coord(new_loc);
        if( action == up ) {
            if( coord.row_ + 1 < nrows_ ) {
                p = new_coord.row_ == coord.row_ + 1 ? pa_ : (new_coord.row_ == coord.row_ ? 1 - pa_ : 0);
            } else {
                assert(coord.row_ + 1 == nrows_);
                p = new_coord.row_ == coord.row_ ? 1 : 0;
            }
        } else if( action == right ) {
            if( coord.col_ + 1 < ncols_ ) {
                p = new_coord.col_ == coord.col_ + 1 ? pa_ : (new_coord.col_ == coord.col_ ? 1 - pa_ : 0);
            } else {
                assert(coord.col_ + 1 == ncols_);
                p = new_coord.col_ == coord.col_ ? 1 : 0;
            }
        } else if( action == down ) {
            if( coord.row_ > 0 ) {
                p = new_coord.row_ + 1 == coord.row_ ? pa_ : (new_coord.row_ == coord.row_ ? 1 - pa_ : 0);
            } else {
                assert(coord.row_ == 0);
                p = new_coord.row_ == coord.row_ ? 1 : 0;
            }
        } else if( action == left ) {
            if( coord.col_ > 0 ) {
                p = new_coord.col_ + 1 == coord.col_ ? pa_ : (new_coord.col_ == coord.col_ ? 1 - pa_ : 0);
            } else {
                assert(coord.col_ == 0);
                p = new_coord.col_ == coord.col_ ? 1 : 0;
            }
        }
        return p;
    }

    // computes P(obs | label at current loc is 'label')
    float obs_probability(int obs, int label, int /*last_action*/) const {
        return label == obs ? po_ : (1 - po_) / float(nlabels_ - 1);
    }

    // computes P(obs | loc, map given by labels)
    float obs_probability(int obs, int loc, const vector<int> &labels, int last_action) const {
        return obs_probability(obs, labels[loc], last_action);
    }

    // run execution: action application and observation sampling
    int regress_loc(int new_loc, int action) const {
        coord_t coord(new_loc), new_coord(new_loc);
        if( action == up )
            coord = coord_t(new_coord.col_, new_coord.row_ == 0 ? 0 : new_coord.row_ - 1);
        else if( action == right )
            coord = coord_t(new_coord.col_ == 0 ? 0 : new_coord.col_ - 1, new_coord.row_);
        else if( action == down )
            coord = coord_t(new_coord.col_, new_coord.row_ == nrows_ - 1 ? nrows_ - 1 : new_coord.row_ + 1);
        else if( action == left )
            coord = coord_t(new_coord.col_ == ncols_ - 1 ? ncols_ - 1 : new_coord.col_ + 1, new_coord.row_);
        return coord.as_index();
    }

    int sample_loc(int loc, int action) const {
        coord_t coord(loc), new_coord(loc);
        if( drand48() < pa_ ) {
            if( action == up )
                new_coord = coord_t(coord.col_, coord.row_ + 1 < nrows_ ? coord.row_ + 1 : coord.row_);
            else if( action == right )
                new_coord = coord_t(coord.col_ + 1 < ncols_ ? coord.col_ + 1 : coord.col_, coord.row_);
            else if( action == down )
                new_coord = coord_t(coord.col_, coord.row_ > 0 ? coord.row_ - 1 : coord.row_);
            else if( action == left )
                new_coord = coord_t(coord.col_ > 0 ? coord.col_ - 1 : coord.col_, coord.row_);
        }
        return new_coord.as_index();
    }

    int sample_obs(int loc, int /*last_action*/) const {
        assert((loc >= 0) && (loc < int(cells_.size())));
        int label = cells_[loc].label_;
        if( drand48() > po_ ) {
            int i = lrand48() % (nlabels_ - 1);
            for( int j = 0; j < nlabels_; ++j ) {
                if( (label != j) && (i == 0) ) {
                    label = j;
                    break;
                } else if( label != j ) {
                    --i;
                }
            }
        }
        return label;
    }

    void initialize(int initial_loc, vector<tracking_t*> &tracking_algorithms) const {
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            tracking_t &tracking = *tracking_algorithms[i];
            tracking.initialize(initial_loc);
            tracking.calculate_marginals();
            tracking.store_marginals();
        }
    }
    void advance_step(int last_action, int obs, vector<tracking_t*> &tracking_algorithms) const {
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            tracking_t &tracking = *tracking_algorithms[i];
            tracking.update(last_action, obs);
            tracking.calculate_marginals();
            tracking.store_marginals();
        }
    }

    struct execution_step_t {
        int loc_;
        int obs_;
        int last_action_;
        execution_step_t(int loc = 0, int obs = 0, int last_action = 0)
          : loc_(loc), obs_(obs), last_action_(last_action) { }
    };
    struct execution_t : public vector<execution_step_t> { };

    void run_execution(const int *labels,
                       int labels_size,
                       const execution_t &input_execution,
                       execution_t &output_execution,
                       int nsteps,
                       const action_selection_t *policy,
                       vector<tracking_t*> &tracking_algorithms) {
        // set labels for callmap
        if( labels_size > 0 )
            set_labels(labels, labels_size);
        else
            sample_labels();

        // initialize tracking algorithms and output execution
        initialize(0, tracking_algorithms); // initial loc is zero. This should be parametrized!
        output_execution.clear();
        output_execution.push_back(execution_step_t(0, -1, -1));

        int hidden_loc = 0;
        for( size_t t = 0; true; ++t ) {
            if( !input_execution.empty() && (t >= input_execution.size()) ) return;
            if( input_execution.empty() && (t >= size_t(nsteps)) ) return;
            int last_action = -1;
            int obs = -1;

            // select next action, update hidden loc, and sample observation
            if( !input_execution.empty() ) {
                hidden_loc = input_execution[t].loc_;
                obs = input_execution[t].obs_;
                last_action = input_execution[t].last_action_;
            } else {
                assert(policy != 0);
                assert(!tracking_algorithms.empty());
                last_action = policy->select_action(tracking_algorithms[0]);
                hidden_loc = sample_loc(hidden_loc, last_action);
                obs = sample_obs(hidden_loc, last_action);
                cerr << "step (t=" << t << "): last_action=" << last_action << ", obs=" << obs << ", loc=" << hidden_loc << endl;
            } 

            // update tracking
            output_execution.push_back(execution_step_t(hidden_loc, obs, last_action));
            advance_step(last_action, obs, tracking_algorithms);
        }
    }
    void run_execution(const int *labels,
                       int labels_size,
                       const execution_t &input_execution,
                       execution_t &output_execution,
                       vector<tracking_t*> &tracking_algorithms) {
        run_execution(labels, labels_size, input_execution, output_execution, 0, 0, tracking_algorithms);
    }
    void run_execution(const int *labels,
                       int labels_size,
                       execution_t &output_execution,
                       int nsteps,
                       const action_selection_t &policy,
                       vector<tracking_t*> &tracking_algorithms) {
        execution_t empty_execution;
        run_execution(labels, labels_size, empty_execution, output_execution, nsteps, &policy, tracking_algorithms);
    }
    void run_execution(const vector<int> &labels,
                       const execution_t &input_execution,
                       execution_t &output_execution,
                       vector<tracking_t*> &tracking_algorithms) {
        run_execution(&labels[0], labels.size(), input_execution, output_execution, 0, 0, tracking_algorithms);
    }
    void run_execution(const vector<int> &labels,
                       execution_t &output_execution,
                       int nsteps,
                       const action_selection_t &policy,
                       vector<tracking_t*> &tracking_algorithms) {
        execution_t empty_execution;
        run_execution(&labels[0], labels.size(), empty_execution, output_execution, nsteps, &policy, tracking_algorithms);
    }

    // scoring of tracking algorithms
    typedef int score_t;

    score_t compute_score(int current_loc, const set<int> &relevant_cells, const tracking_t &tracking) const {
        score_t score = 0;
        for( int var = 0; var < 1 + nrows_ * ncols_; ++var ) {
            if( relevant_cells.find(var) != relevant_cells.end() ) {
                marginal_t marginal;
                tracking.get_marginal(var, marginal);
                float max_probability = 0;
                vector<int> most_probable;
                for( size_t i = 0; i < marginal.size(); ++i ) {
                    if( marginal[i] >= max_probability ) {
                        if( marginal[i] > max_probability )
                            most_probable.clear();
                        most_probable.push_back(i);
                        max_probability = marginal[i];
                    }
                }
                if( most_probable.size() == 1 ) {
                    if( var < nrows_ * ncols_ ) {
                        score += cells_[var].label_ == most_probable[0] ? 1 : 0;
                    } else {
                        score += current_loc == most_probable[0] ? 1 : 0;
                    }
                }
            }
        }
        return score;
    }
    score_t compute_score(const execution_t &execution, const tracking_t &tracking) const {
        set<int> relevant_cells;
        for( size_t i = 0; i < execution.size(); ++i )
            relevant_cells.insert(execution[i].loc_);
        int current_loc = execution.back().loc_;
        return compute_score(current_loc, relevant_cells, tracking);
    }

    void compute_scores(const execution_t &execution, const vector<tracking_t*> &tracking_algorithms, vector<score_t> &scores) const {
        scores.clear();
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            scores.push_back(compute_score(execution, *tracking_algorithms[i]));
    }

    // R plots
    void generate_R_plot(ostream &os, const tracking_t &tracking) const {
        for( size_t t = 0; t < tracking.marginals_.size(); ++t ) {
            os << "mar_" << tracking.name_ << "_t" << t << " <- c(" << tracking.stored_marginal(t, nrows_ * ncols_) << ");" << endl;
        }
        os << "mar_" << tracking.name_ << " <- c(";
        for( size_t t = 0; t < tracking.marginals_.size(); ++t )
            os << "mar_" << tracking.name_ << "_t" << t << (t + 1 < tracking.marginals_.size() ? ", " : "");
        os << ");" << endl;

        os << "dmf <- melt(as.data.frame(t(matrix(mar_" << tracking.name_ << ", ncol=" << ncols_ << ", byrow=T))));" << endl
           << "dmf$y <- rep(1:" << ncols_ << ", " << tracking.marginals_.size() << ");" << endl
           << "plot_" << tracking.name_ << " <- ggplot(dmf,aes(y=variable, x=y)) + "
           << "geom_tile(aes(fill=value)) + "
           << "geom_text(aes(label=ifelse(value>.01, trunc(100*value, digits=2)/100, \"\")), size=3, angle=0, color=\"white\") +"
           << "labs(y=\"time step\", x=\"location\") + "
           << "theme_minimal() + "
           << "scale_x_discrete(limits=1:" << ncols_ << ") + "
           << "scale_y_discrete(labels=paste(\"t=\", 0:" << tracking.marginals_.size() << ", sep=\"\"));" << endl;

        os << "pdf(\"marginals_" << tracking.name_ << ".pdf\");" << endl
           << "show(plot_" << tracking.name_ << ")" << endl
           << "dev.off();" << endl;
    }
};

tracking_t::tracking_t(const string &name, const cellmap_t &cellmap)
  : name_(name), cellmap_(cellmap) {
    nloc_ = cellmap_.nrows_ * cellmap_.ncols_;
    nlabels_ = cellmap_.nlabels_;
}

struct naive_action_selection_t : public action_selection_t {
    naive_action_selection_t(const cellmap_t &cellmap) : action_selection_t(cellmap) { }
    virtual ~naive_action_selection_t() { }
    virtual int select_action(const tracking_t */*tracking*/) const {
        int action = drand48() > .30 ? cellmap_t::right : cellmap_t::left;
        return action;
    }
};


// (Standard) Probabilistic Causal Belief Tracking
struct pcbt_t : public tracking_t {
    int memory_;
    int prefix_mask_;
    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;

    bool project_join_;
    bool marginals_calculated_;

    string algorithm_;
    dai::PropertySet opts_;

    mutable dai::FactorGraph fg_;
    mutable dai::InfAlg *inference_algorithm_;

    pcbt_t(const string &name, const cellmap_t &cellmap, int memory, bool project_join = false)
      : tracking_t(name, cellmap), memory_(memory), project_join_(project_join)  {
        prefix_mask_ = 1;
        for( int i = 0; i < memory_; ++i ) prefix_mask_ *= nloc_;
        //cerr << "pcbt: memory=" << memory_ << ", prefix=" << prefix_mask_ << endl;
        inference_algorithm_ = 0;
        marginals_calculated_ = false;
        create_variables_and_factors();
    }
    virtual ~pcbt_t() { }

    void create_variables_and_factors() {
        variables_ = vector<dai::Var>(cellmap_.nrows_ * cellmap_.ncols_ + 1);
        for( int i = 0; i < nloc_; ++i )
            variables_[i] = dai::Var(i, nlabels_);
        variables_[nloc_] = dai::Var(nloc_, prefix_mask_ * nloc_);
        factors_ = vector<dai::Factor>(nloc_);
    }

    void reset_factors(size_t initial_loc) {
        int init_loc_hist = initial_loc;
        for( int i = 0; i < memory_; ++i )
            init_loc_hist = init_loc_hist * nloc_ + initial_loc;

        for( int i = 0; i < nloc_; ++i ) {
            dai::VarSet factor_vars(variables_[i], variables_[nloc_]);
            factors_[i] = dai::Factor(factor_vars, 0.0);

            // set valuations for initial loc (memory loc variables are also set to initial loc)
            for( int label = 0; label < nlabels_; ++label )
                factors_[i].set(init_loc_hist * nlabels_ + label, 1.0);
        }
    }

    void progress_factor_with_action(int /*findex*/, dai::Factor &factor, int last_action) const {
        //cerr << "    PROGRESS[action=" << last_action << "]:"
        //     << " old factor[" << findex << "]: " << factor << endl;
        if( last_action != -1 ) {
            dai::Factor new_factor(factor.vars(), 0.0);
            for( size_t j = 0; j < factor.nrStates(); ++j ) {
                int label = j % nlabels_;
                int loc_hist = j / nlabels_;
                int loc = loc_hist % nloc_;
                int prefix_loc_hist = loc_hist % prefix_mask_;
                float weight = factor[j];
                for( int new_loc = 0; new_loc < nloc_; ++new_loc ) {
                    int new_loc_hist = prefix_loc_hist * nloc_ + new_loc;
                    float new_weight = weight * cellmap_.loc_probability(last_action, loc, new_loc);
                    int index = new_loc_hist * nlabels_ + label;
                    assert(index < int(new_factor.nrStates()));
                    if( new_weight != 0 ) new_factor.set(index, new_factor[index] + new_weight);
                }
            }
            factor = new_factor;
        }
        //cerr << "    PROGRESS[action=" << last_action << "]:"
        //     << " new factor[" << findex << "]: " << factor << endl;
    }

    void filter_factor_with_obs(int findex, dai::Factor &factor, int last_action, int obs) const {
        //cerr << "    FILTER[obs=" << obs << "]:"
        //     << " old factor[" << findex << "]: " << factor << endl;
        if( obs != -1 ) {
            for( size_t j = 0; j < factor.nrStates(); ++j ) {
                float weight = factor[j];
                int label = j % nlabels_;
                int loc_hist = j / nlabels_;
                if( findex == (loc_hist % nloc_) ) { // findex == "most recent loc in loc_hist"
                    factor.set(j, weight * cellmap_.obs_probability(obs, label, last_action));
                } else {
                    factor.set(j, weight / float(nlabels_));
                }
            }
        }
        //cerr << "    FILTER[obs=" << obs << "]:"
        //     << " new factor[" << findex << "]: " << factor << endl;
    }

    void marginalize_beams() {
        for( int i = 0; i < int(factors_.size()); ++i ) {
            cerr << "MARGINALIZE: old factor[" << i << "]: " << factors_[i] << endl;
            dai::Factor &factor = factors_[i];
            const dai::Factor &new_factor = inference_algorithm_->belief(factor.vars());
            factor = new_factor;
            cerr << "MARGINALIZE: new factor[" << i << "]: " << factors_[i] << endl;
        }
    }

    void update_factor(int i, int last_action, int obs) {
        //cerr << "UPDATE[last_action=" << last_action << ", obs=" << obs << "]:"
        //     << " old factor[" << i << "]: " << factors_[i] << endl;
        progress_factor_with_action(i, factors_[i], last_action);
        filter_factor_with_obs(i, factors_[i], last_action, obs);
        //cerr << "UPDATE[last_action=" << last_action << ", obs=" << obs << "]:"
        //     << " new factor[" << i << "]: " << factors_[i] << endl;
    }

    void set_algorithm_and_options(const string &algorithm, const dai::PropertySet &opts) {
        algorithm_ = algorithm;
        opts_ = opts;
    }

    virtual void initialize(int initial_loc) {
        marginals_.clear();
        reset_factors(initial_loc);
    }

    virtual void update(int last_action, int obs) {
        assert((obs >= 0) && (obs < nlabels_));
        for( int i = 0; i < int(factors_.size()); ++i )
            update_factor(i, last_action, obs);
        marginals_calculated_ = false;

        if( project_join_ ) {
            calculate_marginals_internal();
            marginalize_beams();
        }
    }

    void calculate_marginals_internal() {
        fg_ = dai::FactorGraph(factors_);
        delete inference_algorithm_;
        if( algorithm_ == "JT" )
            inference_algorithm_ = new dai::JTree(fg_, opts_);
        else if( algorithm_ == "BP" )
            inference_algorithm_ = new dai::BP(fg_, opts_);
        else if( algorithm_ == "HAK" )
            inference_algorithm_ = new dai::HAK(fg_, opts_);
        inference_algorithm_->init();
        inference_algorithm_->run();
        marginals_calculated_ = true;
    }
    virtual void calculate_marginals() {
        if( !marginals_calculated_ ) calculate_marginals_internal();
        if( project_join_ ) marginalize_beams();
    }

    virtual void get_marginal(int var, marginal_t &marginal) const {
        if( !project_join_ ) {
            const dai::Factor &factor = inference_algorithm_->belief(fg_.var(var));
            if( var < nloc_ ) {
                marginal = marginal_t(factor.nrStates());
                for( size_t i = 0; i < factor.nrStates(); ++i )
                    marginal[i] = factor[i];
            } else {
                marginal = marginal_t(nloc_, 0);
                for( size_t i = 0; i < factor.nrStates(); ++i )
                    marginal[i % nloc_] += factor[i];
            }
        } else {
            if( var < nloc_ ) {
                const dai::Factor &factor = factors_[var];
                marginal = marginal_t(factor.nrStates());
                for( size_t i = 0; i < factor.nrStates(); ++i )
                    marginal[i] = factor[i];
            } else {
                const dai::Factor &factor = factors_[0];
                marginal = marginal_t(nloc_, 0);
                for( size_t i = 0; i < factor.nrStates(); ++i ) {
                    int loc_hist = i / nlabels_;
                    marginal[loc_hist % nloc_] += factor[i];
                }
            }
        }
    }
};



// Generic Particle Filter
template <typename T> struct PF_t : public tracking_t {
    int nparticles_;
    vector<pair<float, T> > particles_;
    vector<marginal_t> marginals_on_vars_;

    PF_t(const string &name, const cellmap_t &cellmap, int nparticles)
      : tracking_t(name, cellmap), nparticles_(nparticles) {
    }
    virtual ~PF_t() { }

    virtual void initialize(int initial_loc) = 0;
    virtual void update(int last_action, int obs) = 0;
    virtual void calculate_marginals() = 0;
    virtual void get_marginal(int var, marginal_t &marginal) const = 0;

    void stochastic_sampling(int n, vector<int> &indices) const {
        marginal_t cdf(nparticles_, 0);
        cdf[0] = particles_[0].first;
        for( int i = 1; i < nparticles_; ++i )
            cdf[i] = cdf[i - 1] + particles_[i].first;

        indices.clear();
        indices.reserve(n);
        for( int j = 0; j < n; ++j ) {
            float u = drand48();
            if( u > cdf.back() ) {
                indices.push_back(nparticles_ - 1);
            } else {
                for( size_t i = 0; i < cdf.size(); ++i ) {
                    if( u < cdf[i] ) {
                        indices.push_back(i);
                        break;
                    }
                }
            }
        }
    }

    void stochastic_universal_sampling(int n, vector<int> &indices) const {
        marginal_t cdf(nparticles_, 0);
        cdf[0] = particles_[0].first;
        for( int i = 1; i < nparticles_; ++i )
            cdf[i] = cdf[i - 1] + particles_[i].first;

        indices.clear();
        indices.reserve(n);
        float u = drand48() / float(n);
        for( int i = 0, j = 0; j < n; ++j ) {
            while( (i < nparticles_) && (u > cdf[i]) ) ++i;
            indices.push_back(i == nparticles_ ? nparticles_ - 1 : i);
            u += 1.0 / float(n);
        }
    }

    int sample_from_distribution(int n, const float *cdf) const {
        float p = drand48();
        for( int i = 0; i < n; ++i )
            if( p < cdf[i] ) return i;
        return n - 1;
    }
};

// Sequential Importance Sampling (SIS) Particle Filter
struct SIS_t : public PF_t<vector<int> > {
    SIS_t(const string &name, const cellmap_t &cellmap, int nparticles)
      : PF_t(name, cellmap, nparticles) {
    }
    virtual ~SIS_t() { }

    virtual void initialize(int initial_loc) {
        particles_ = vector<pair<float, vector<int> > >(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1;
            particles_[i].second = vector<int>(nloc_ + 1);
            for( int j = 0; j < nloc_; ++j )
                particles_[i].second[j] = lrand48() % nlabels_;
            particles_[i].second[nloc_] = initial_loc;
        }
    }

    virtual void update(int last_action, int obs) {
        for( int i = 0; i < nparticles_; ++i ) {
            float &weight = particles_[i].first;
            vector<int> &p = particles_[i].second;
            int new_loc = cellmap_.sample_loc(p[nloc_], last_action);
            weight *= cellmap_.obs_probability(obs, p[nloc_], p, last_action);
            p[nloc_] = new_loc;
        }
    }

    virtual void calculate_marginals() {
        marginals_on_vars_ = vector<marginal_t>(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            marginals_on_vars_[i] = marginal_t(nlabels_, 0.0);
        marginals_on_vars_[nloc_] = marginal_t(nloc_, 0.0);

        float total_mass = 0;
        for( int i = 0; i < nparticles_; ++i )
            total_mass += particles_[i].first;

        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const vector<int> &p = particles_[i].second;
            for( int j = 0; j <= nloc_; ++j )
                marginals_on_vars_[j][p[j]] += weight / total_mass;
        }
    }

    virtual void get_marginal(int var, marginal_t &marginal) const {
        marginal = marginals_on_vars_[var];
    }
};

// Generic Sequential Importance Resampling (SIR) Particle Filter
struct SIR_t : public PF_t<vector<int> > {
    vector<pair<int, int> > history_;

    SIR_t(const string &name, const cellmap_t &cellmap, int nparticles)
      : PF_t(name, cellmap, nparticles) {
    }
    virtual ~SIR_t() { }

    virtual void sample_from_pi(vector<int> &np, const vector<int> &p, int last_action, int obs) const = 0;
    virtual float importance_weight(const vector<int> &np, const vector<int> &p, int last_action, int obs) const = 0;

    virtual void initialize(int initial_loc) {
        particles_ = vector<pair<float, vector<int> > >(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1.0 / float(nparticles_);
            particles_[i].second = vector<int>(nloc_ + 1);
            for( int j = 0; j < nloc_; ++j )
                particles_[i].second[j] = lrand48() % nlabels_;
            particles_[i].second[nloc_] = initial_loc;
        }
    }

    virtual void update(int last_action, int obs) {
        vector<int> indices;
        stochastic_universal_sampling(nparticles_, indices);
        assert(indices.size() == size_t(nparticles_));

        float total_mass = 0.0;
        vector<pair<float, vector<int> > > new_particles;
        new_particles.reserve(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            int index = indices[i];
            vector<int> &p = particles_[index].second;
            vector<int> np;
            sample_from_pi(np, p, last_action, obs);
            float weight = importance_weight(np, p, last_action, obs);
            total_mass += weight;
            new_particles.push_back(make_pair(weight, np));
        }
        for( int i = 0; i < nparticles_; ++i )
            new_particles[i].first /= total_mass;
        particles_ = new_particles;

        history_.push_back(make_pair(last_action, obs));
    }

    virtual void calculate_marginals() {
        marginals_on_vars_ = vector<marginal_t>(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            marginals_on_vars_[i] = marginal_t(nlabels_, 0.0);
        marginals_on_vars_[nloc_] = marginal_t(nloc_, 0.0);

        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const vector<int> &p = particles_[i].second;
            for( int j = 0; j < nloc_; ++j )
                marginals_on_vars_[j][p[j]] += weight;
            marginals_on_vars_[nloc_][p.back()] += weight;
        }
    }

    virtual void get_marginal(int var, marginal_t &marginal) const {
        marginal = marginals_on_vars_[var];
    }
};

struct optimal_SIR_t : public SIR_t {
    int nstates_;
    float **cdf_;

    optimal_SIR_t(const string &name, const cellmap_t &cellmap, int nparticles)
      : SIR_t(name, cellmap, nparticles) {
        nstates_ = nloc_;
        for( int loc = 0; loc < nloc_; ++loc )
            nstates_ *= nlabels_;
        calculate_cdf_for_pi();
    }
    virtual ~optimal_SIR_t() {
        for( int i = 0; i < nloc_ * nlabels_ * 4; ++i )
            delete[] cdf_[i];
        delete[] cdf_;
    }

    void calculate_cdf_for_pi() {
        cdf_ = new float*[nstates_ * nlabels_ * 4];
        for( int i = 0; i < nstates_ * nlabels_ * 4; ++i ) {
            cdf_[i] = new float[nstates_];
            vector<int> state;
            int state_index = i % nstates_;
            int obs = (i / nstates_) % nlabels_;
            int action = (i / nstates_) / nlabels_;
            decode(state_index, state);
            for( int state_index2 = 0; state_index2 < nstates_; ++state_index2 ) {
                vector<int> state2;
                decode(state_index2, state2);
                cdf_[i][state_index2] = state_index2 > 0 ? cdf_[i][state_index2 - 1] : 0;
                cdf_[i][state_index2] += pi(state2, state, action, obs);
            }
            //cerr << "cdf[" << i << "/" << nstates_ * nlabels_ * 4 << "][" << nstates_ - 1 << "]="
            //     << setprecision(9) << cdf_[i][nstates_ - 1] << endl;
            assert(fabs(cdf_[i][nstates_ - 1] - 1.0) < .0001);
        }
    }
    const float* pi_cdf(int state_index, int last_action, int obs) const {
        return cdf_[(last_action * nlabels_ + obs) * nstates_ + state_index];
    }

    bool same_labels(const vector<int> &state1, const vector<int> &state2) const {
        for( int loc = 0; loc < nloc_; ++loc )
            if( state1[loc] != state2[loc] ) return false;
        return true;
    }
    int encode(const vector<int> &state) const {
        int state_index = 0;
        for( int loc = 0; loc < nloc_; ++loc ) {
            state_index *= nlabels_;
            state_index += state[loc];
        }
        state_index = state_index * nloc_ + state[nloc_];
        return state_index;
    }
    void decode(int state_index, vector<int> &state) const {
        state = vector<int>(nloc_ + 1, 0);
        state[nloc_] = state_index % nloc_;
        state_index /= nloc_;
        for( int loc = nloc_ - 1; loc >= 0; --loc ) {
            state[loc] = state_index % nlabels_;
            state_index /= nlabels_;
        }
    }

    virtual void sample_from_pi(vector<int> &new_state, const vector<int> &state, int last_action, int obs) const {
        const float *dist = pi_cdf(encode(state), last_action, obs);
        int state_index = sample_from_distribution(nstates_, dist);
        decode(state_index, new_state);
    }

    float pi(const vector<int> &new_state, const vector<int> &state, int last_action, int obs) const {
        // new_state has probability:
        //   P(new_state|state,last_action,obs) = P(new_state,obs|state,last_action) / P(obs|state,last_action)
        //                                      = P(obs|new_state,state,last_action) * P(new_state|state,last_action) / P(obs|state,last_action)
        //                                      = P(obs|new_state,last_action) * P(new_state|state,last_action) / P(obs|state,last_action)
        if( same_labels(new_state, state) ) {
            float p = cellmap_.obs_probability(obs, new_state[nloc_], new_state, last_action);
            p *= cellmap_.loc_probability(last_action, state[nloc_], new_state[nloc_]);
            return p / importance_weight(new_state, state, last_action, obs);
        } else {
            return 0;
        }
    }

    virtual float importance_weight(const vector<int> &/*new_state*/, const vector<int> &state, int last_action, int obs) const {
        // weight = P(obs|new_state,last_action) * P(new_state|state,last_action) / P(new_state|state,last_action,obs)
        //        = P(obs|state,last_action) [see above derivation in pi(..)]
        //        = SUM P(new_state,obs|state,last_action)
        //        = SUM P(obs|new_state,state,last_action) P(new_state|state,last_action)
        //        = SUM P(obs|new_state,last_action) P(new_state|state,last_action)
        // since P(new_state|state,last_action) = 0 if state and new_states have different labels,
        // we can simplify the sum over locations instead of states
        float p = 0;
        int loc = state[nloc_];
        for( int new_loc = 0; new_loc < nloc_; ++new_loc )
            p += cellmap_.obs_probability(obs, new_loc, state, last_action) * cellmap_.loc_probability(last_action, loc, new_loc);
        return p;
    }
};

struct motion_model_SIR_t : public SIR_t {
    motion_model_SIR_t(const string &name, const cellmap_t &cellmap, int nparticles)
      : SIR_t(name, cellmap, nparticles) {
    }
    virtual ~motion_model_SIR_t() { }

    virtual void sample_from_pi(vector<int> &np, const vector<int> &p, int last_action, int /*obs*/) const {
        np = p;
        np[nloc_] = cellmap_.sample_loc(p[nloc_], last_action);
    }
    virtual float importance_weight(const vector<int> &np, const vector<int> &/*p*/, int last_action, int obs) const {
        return cellmap_.obs_probability(obs, np[nloc_], np, last_action);
    }
};


// Generic Sequential Importance Resampling (SIR2) Particle Filter
template <typename T> struct SIR2_t : public PF_t<T> {
    using PF_t<T>::nloc_;
    using PF_t<T>::nlabels_;
    using PF_t<T>::particles_;
    using PF_t<T>::nparticles_;
    using PF_t<T>::marginals_on_vars_;

    vector<pair<int, int> > history_;

    SIR2_t(const string &name, const cellmap_t &cellmap, int nparticles)
      : PF_t<T>(name, cellmap, nparticles) {
    }
    virtual ~SIR2_t() { }

    virtual void sample_from_pi(T &np, const T &p, int last_action, int obs) const = 0;
    virtual float importance_weight(const T &np, const T &p, int last_action, int obs) const = 0;

    virtual void initialize(int initial_loc) {
        particles_ = vector<pair<float, T> >(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1.0 / float(nparticles_);
            particles_[i].second.sample(nloc_, nlabels_, initial_loc);
        }
    }

    virtual void update(int last_action, int obs) {
        vector<int> indices;
        PF_t<T>::stochastic_universal_sampling(nparticles_, indices);
        //PF_t<T>::stochastic_sampling(nparticles_, indices);
        assert(indices.size() == size_t(nparticles_));

        float total_mass = 0.0;
        vector<pair<float, T> > new_particles;
        new_particles.reserve(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            int index = indices[i];
            const T &p = particles_[index].second;
            T np;
            sample_from_pi(np, p, last_action, obs);
            float weight = importance_weight(np, p, last_action, obs);
            total_mass += weight;
            new_particles.push_back(make_pair(weight, np));
        }
        for( int i = 0; i < nparticles_; ++i )
            new_particles[i].first /= total_mass;
        particles_ = new_particles;

        history_.push_back(make_pair(last_action, obs));
    }

    virtual void calculate_marginals() {
        marginals_on_vars_ = vector<marginal_t>(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            marginals_on_vars_[i] = marginal_t(nlabels_, 0.0);
        marginals_on_vars_[nloc_] = marginal_t(nloc_, 0.0);
        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const T &p = particles_[i].second;
            for( int j = 0; j < nloc_; ++j )
                marginals_on_vars_[j][p.label_on_loc(j)] += weight;
            marginals_on_vars_[nloc_][p.current_loc()] += weight;
        }
    }

    virtual void get_marginal(int var, marginal_t &marginal) const {
        marginal = marginals_on_vars_[var];
    }
};


// Base particle used in the different SIR filters
struct base_particle_t {
    int current_loc_;
    vector<int> map_;

    void sample(int nloc, int nlabels, int initial_loc) {
        map_ = vector<int>(nloc);
        for( int i = 0; i < nloc; ++i )
            map_[i] = lrand48() % nlabels;
        current_loc_ = initial_loc;
    }
    int label_on_loc(int loc) const { return map_[loc]; }
    int current_loc() const { return current_loc_; }
};


// SIR filter with proposal distribution given by optimal choice (intractable)
struct optimal_SIR2_t : public SIR2_t<base_particle_t> {
    int nstates_;
    float **cdf_;

    optimal_SIR2_t(const string &name, const cellmap_t &cellmap, int nparticles)
      : SIR2_t(name, cellmap, nparticles) {
        nstates_ = nloc_;
        for( int loc = 0; loc < nloc_; ++loc )
            nstates_ *= nlabels_;
        calculate_cdf_for_pi();
    }
    virtual ~optimal_SIR2_t() {
        for( int i = 0; i < nloc_ * nlabels_ * 4; ++i )
            delete[] cdf_[i];
        delete[] cdf_;
    }

    void calculate_cdf_for_pi() {
        cdf_ = new float*[nstates_ * nlabels_ * 4];
        for( int i = 0; i < nstates_ * nlabels_ * 4; ++i ) {
            cdf_[i] = new float[nstates_];
            base_particle_t p;
            int state_index = i % nstates_;
            int obs = (i / nstates_) % nlabels_;
            int action = (i / nstates_) / nlabels_;
            decode(state_index, p);
            for( int state_index2 = 0; state_index2 < nstates_; ++state_index2 ) {
                base_particle_t p2;
                decode(state_index2, p2);
                cdf_[i][state_index2] = state_index2 > 0 ? cdf_[i][state_index2 - 1] : 0;
                cdf_[i][state_index2] += pi(p2, p, action, obs);
            }
            //cerr << "cdf[" << i << "/" << nstates_ * nlabels_ * 4 << "][" << nstates_ - 1 << "]="
            //     << setprecision(9) << cdf_[i][nstates_ - 1] << endl;
            assert(fabs(cdf_[i][nstates_ - 1] - 1.0) < .0001);
        }
    }
    const float* pi_cdf(int state_index, int last_action, int obs) const {
        return cdf_[(last_action * nlabels_ + obs) * nstates_ + state_index];
    }

    int encode(const base_particle_t &p) const {
        int state_index = 0;
        for( int loc = 0; loc < nloc_; ++loc ) {
            state_index *= nlabels_;
            state_index += p.map_[loc];
        }
        state_index = state_index * nloc_ + p.current_loc();
        return state_index;
    }
    void decode(int state_index, base_particle_t &p) const {
        p.map_ = vector<int>(nloc_, 0);
        p.current_loc_ = state_index % nloc_;
        state_index /= nloc_;
        for( int loc = nloc_ - 1; loc >= 0; --loc ) {
            p.map_[loc] = state_index % nlabels_;
            state_index /= nlabels_;
        }
    }

    virtual void sample_from_pi(base_particle_t &np, const base_particle_t &p, int last_action, int obs) const {
        const float *dist = pi_cdf(encode(p), last_action, obs);
        int state_index = sample_from_distribution(nstates_, dist);
        decode(state_index, np);
    }

    float pi(const base_particle_t &np, const base_particle_t &p, int last_action, int obs) const {
        // np has probability:
        //   P(np|p,last_action,obs) = P(np,obs|p,last_action) / P(obs|p,last_action)
        //                           = P(obs|np,p,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        //                           = P(obs|np,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        if( np.map_ == p.map_ ) {
            float prob = cellmap_.obs_probability(obs, np.current_loc(), np.map_, last_action);
            prob *= cellmap_.loc_probability(last_action, p.current_loc(), np.current_loc());
            return prob / importance_weight(np, p, last_action, obs);
        } else {
            return 0;
        }
    }

    virtual float importance_weight(const base_particle_t &/*np*/, const base_particle_t &p, int last_action, int obs) const {
        // weight = P(obs|np,last_action) * P(np|p,last_action) / P(np|p,last_action,obs)
        //        = P(obs|p,last_action) [see above derivation in pi(..)]
        //        = SUM P(np,obs|p,last_action)
        //        = SUM P(obs|np,p,last_action) P(np|p,last_action)
        //        = SUM P(obs|np,last_action) P(np|p,last_action)
        // since P(np|p,last_action) = 0 if p and np have different labels,
        // we can simplify the sum over locations instead of maps
        float weight = 0;
        int loc = p.current_loc();
        for( int new_loc = 0; new_loc < nloc_; ++new_loc )
            weight += cellmap_.obs_probability(obs, new_loc, p.map_, last_action) * cellmap_.loc_probability(last_action, loc, new_loc);
        return weight;
    }
};


// SIR filter with proposal distribution given by motion model
struct motion_model_SIR2_t : public SIR2_t<base_particle_t> {
motion_model_SIR2_t(const string &name, const cellmap_t &cellmap, int nparticles)
: SIR2_t(name, cellmap, nparticles) {
}
virtual ~motion_model_SIR2_t() { }

virtual void sample_from_pi(base_particle_t &np, const base_particle_t &p, int last_action, int /*obs*/) const {
np = p;
np.current_loc_ = cellmap_.sample_loc(p.current_loc(), last_action);
}
virtual float importance_weight(const base_particle_t &np, const base_particle_t &/*p*/, int last_action, int obs) const {
return cellmap_.obs_probability(obs, np.current_loc(), np.map_, last_action);
}
};


// Rao-Blackwellised SIR Filter
struct RBPF_particle_t : public base_particle_t {
    vector<marginal_t> factors_;
    void sample(int nloc, int nlabels, int initial_loc) {
        //base_particle_t::sample(nloc, nlabels, initial_loc);
        current_loc_ = initial_loc;
        factors_ = vector<marginal_t>(nloc, marginal_t(nlabels, 1.0 / float(nlabels)));
    }
    void update(int last_action, int obs, const cellmap_t &cellmap) {
        assert(current_loc() < int(factors_.size()));
        marginal_t &factor = factors_[current_loc()];
        assert(cellmap.nlabels_ == int(factor.size()));
        float total_mass = 0.0;
        for( int label = 0; label < cellmap.nlabels_; ++label ) {
            factor[label] *= cellmap.obs_probability(obs, label, last_action);
            total_mass += factor[label];
        }
        for( int label = 0; label < cellmap.nlabels_; ++label )
            factor[label] /= total_mass;
    }
    float probability(int label, int loc) const {
        assert(loc < int(factors_.size()));
        assert(label < int(factors_[loc].size()));
        return factors_[loc][label];
    }

    const RBPF_particle_t& operator=(const RBPF_particle_t &p) {
        current_loc_ = p.current_loc_;
        factors_ = p.factors_;
        return *this;
    }
};

struct RBPF2_t : public SIR2_t<RBPF_particle_t> {
    RBPF2_t(const string &name, const cellmap_t &cellmap, int nparticles)
      : SIR2_t(name, cellmap, nparticles) {
    }
    virtual ~RBPF2_t() { }

    virtual void sample_from_pi(RBPF_particle_t &np, const RBPF_particle_t &p, int last_action, int obs) const {
        np = p;
        np.current_loc_ = cellmap_.sample_loc(p.current_loc(), last_action);
        np.update(last_action, obs, cellmap_);
    }
    virtual float importance_weight(const RBPF_particle_t &np, const RBPF_particle_t &p, int last_action, int obs) const {
        float prob = 0;
        for( int label = 0; label < nlabels_; ++label ) // marginalize over possible labels at current loc
            prob += cellmap_.obs_probability(obs, label, last_action) * p.probability(label, np.current_loc());
        return prob;
    }

    virtual void calculate_marginals() {
        marginals_on_vars_ = vector<marginal_t>(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            marginals_on_vars_[i] = marginal_t(nlabels_, 0.0);
        marginals_on_vars_[nloc_] = marginal_t(nloc_, 0.0);
        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const RBPF_particle_t &p = particles_[i].second;
            for( int loc = 0; loc < nloc_; ++loc ) {
                for( int label = 0; label < nlabels_; ++label )
                    marginals_on_vars_[loc][label] += weight * p.probability(label, loc);
            }
            marginals_on_vars_[nloc_][p.current_loc()] += weight;
        }
    }
};


// (Particle) Probabilistic Causal Belief Tracking
struct particle_pcbt_t {
    const cellmap_t *cellmap_;
    int nloc_;
    int nlabels_;
    int current_loc_;
    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;

    mutable dai::FactorGraph fg_;
    mutable dai::InfAlg *inference_algorithm_;

    particle_pcbt_t(const cellmap_t *cellmap) : cellmap_(cellmap) {
        nloc_ = cellmap_->nrows_ * cellmap_->ncols_;
        nlabels_ = cellmap_->nlabels_;
        current_loc_ = 0;
        inference_algorithm_ = 0;
        create_variables_and_factors();
    }

    void create_variables_and_factors() {
        variables_ = vector<dai::Var>(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            variables_[i] = dai::Var(i, nlabels_);
        variables_[nloc_] = dai::Var(nloc_, nloc_);
        factors_ = vector<dai::Factor>(nloc_);
    }

    void reset_factors(size_t initial_loc) {
        current_loc_ = initial_loc;
        for( int i = 0; i < nloc_; ++i ) {
            dai::VarSet factor_vars(variables_[i], variables_[nloc_]);
            factors_[i] = dai::Factor(factor_vars, 0.0);

            // set valuations for initial loc
            for( int label = 0; label < nlabels_; ++label )
                factors_[i].set(initial_loc * nlabels_ + label, 1 / float(nlabels_));
        }
    }

    void progress_factor_with_action(int findex, dai::Factor &factor, int last_action) const {
        //cerr << "    PROGRESS[action=" << last_action << "]:"
        //     << " old factor[" << findex << "]: " << factor << endl;
        if( last_action != -1 ) {
            float total_mass = 0;
            dai::Factor new_factor(factor.vars(), 0.0);
            for( size_t j = 0; j < factor.nrStates(); ++j ) {
                int label = j % nlabels_;
                int loc = j / nlabels_;
                float weight = factor[j];
                if( loc != current_loc_ ) continue;
                for( int new_loc = 0; new_loc < nloc_; ++new_loc ) {
                    float new_weight = weight * cellmap_->loc_probability(last_action, loc, new_loc);
                    int index = new_loc * nlabels_ + label;
                    assert(index < int(new_factor.nrStates()));
                    if( new_weight != 0 ) new_factor.set(index, new_factor[index] + new_weight);
                    total_mass += new_weight;
                }
            }
            new_factor /= total_mass;
            factor = new_factor;
        }
        //cerr << "    PROGRESS[action=" << last_action << "]:"
        //     << " new factor[" << findex << "]: " << factor << endl;
    }

    void filter_factor_with_obs(int findex, dai::Factor &factor, int last_action, int obs) const {
        //cerr << "    FILTER[obs=" << obs << "]:"
        //     << " old factor[" << findex << "]: " << factor << endl;
        if( obs != -1 ) {
            float total_mass = 0;
            for( size_t j = 0; j < factor.nrStates(); ++j ) {
                float weight = factor[j];
                int label = j % nlabels_;
                int loc = j / nlabels_;
                if( findex == loc ) {
                    factor.set(j, weight * cellmap_->obs_probability(obs, label, last_action));
                } else {
                    factor.set(j, weight / float(nlabels_));
                }
                total_mass += factor[j];
            }
            factor /= total_mass;
        }
        //cerr << "    FILTER[obs=" << obs << "]:"
        //     << " new factor[" << findex << "]: " << factor << endl;
    }

    void update_factor(int i, int last_action, int obs) {
        //cerr << "UPDATE[last_action=" << last_action << ", obs=" << obs << "]:"
        //     << " old factor[" << i << "]: " << factors_[i] << endl;
        progress_factor_with_action(i, factors_[i], last_action);
        filter_factor_with_obs(i, factors_[i], last_action, obs);
        //cerr << "UPDATE[last_action=" << last_action << ", obs=" << obs << "]:"
        //     << " new factor[" << i << "]: " << factors_[i] << endl;
    }

    int new_loc(int last_action, int /*obs*/) const {
        return cellmap_->sample_loc(current_loc_, last_action);
    }

    void update(int last_action, int obs, int new_loc) {
        assert((obs >= 0) && (obs < nlabels_));
        for( int i = 0; i < int(factors_.size()); ++i )
            update_factor(i, last_action, obs);
        current_loc_ = new_loc;
    }

    void calculate_marginals(const string &algorithm, const dai::PropertySet &opts) {
        fg_ = dai::FactorGraph(factors_);
        //delete inference_algorithm_;
        if( algorithm == "JT" )
            inference_algorithm_ = new dai::JTree(fg_, opts);
        else if( algorithm == "BP" )
            inference_algorithm_ = new dai::BP(fg_, opts);
        else if( algorithm == "HAK" )
            inference_algorithm_ = new dai::HAK(fg_, opts);
        inference_algorithm_->init();
        inference_algorithm_->run();
    }

    void get_marginal(int var, marginal_t &marginal) const {
        const dai::Factor &factor = inference_algorithm_->belief(fg_.var(var));
        if( var < nloc_ ) {
            marginal = marginal_t(factor.nrStates());
            for( size_t i = 0; i < factor.nrStates(); ++i ) {
                assert(factor[i] <= 1);
                marginal[i] = factor[i];
            }
        } else {
            marginal = marginal_t(nloc_, 0);
            for( size_t i = 0; i < factor.nrStates(); ++i ) {
                assert(factor[i] <= 1);
                marginal[i % nloc_] += factor[i];
            }
        }
    }

    float probability(int label, int loc) const {
        float p = 0;
        const dai::Factor &factor = factors_[loc];
        for( size_t j = 0; j < factor.nrStates(); ++j ) {
            if( j % nlabels_ == label ) p += factor[j];
        }
        return p;
    }
};

struct ppcbt_t : public tracking_t {
    int nparticles_;
    vector<pair<float, particle_pcbt_t> > particles_;

    string algorithm_;
    dai::PropertySet opts_;

    ppcbt_t(const string &name, const cellmap_t &cellmap, int nparticles)
      : tracking_t(name, cellmap), nparticles_(nparticles) {
        create_particles();
    }
    virtual ~ppcbt_t() { }

    void set_algorithm_and_options(const string &algorithm, const dai::PropertySet &opts) {
        algorithm_ = algorithm;
        opts_ = opts;
    }

    void create_particles() {
        for( int i = 0; i < nparticles_; ++i )
            particles_.push_back(make_pair(1, particle_pcbt_t(&cellmap_)));
    }

    virtual void initialize(int initial_loc) {
        marginals_.clear();
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1 / float(nparticles_);
            particles_[i].second.reset_factors(initial_loc);
        }
    }

    virtual void update(int last_action, int obs) {
        float total_weight = 0;
        for( int i = 0; i < nparticles_; ++i ) {
            particle_pcbt_t &p = particles_[i].second;
            int new_loc = p.new_loc(last_action, obs);

            float weight = 0;
            for( int label = 0; label < nlabels_; ++label ) { // marginalize over possible labels at current loc
                weight += cellmap_.obs_probability(obs, label, last_action) * p.probability(label, new_loc);
            }
            particles_[i].first = weight;
            p.update(last_action, obs, new_loc);
            total_weight += weight;
        }
        for( int i = 0; i < nparticles_; ++i )
            particles_[i].first /= total_weight;
    }

    virtual void calculate_marginals() {
        for( size_t i = 0; i < particles_.size(); ++i )
            particles_[i].second.calculate_marginals(algorithm_, opts_);
    }

    virtual void get_marginal(int var, marginal_t &marginal) const {
        marginal_t pmarginal;
        marginal = marginal_t(var < nloc_ ? nlabels_ : nloc_);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].second.get_marginal(var, pmarginal);
            pmarginal *= particles_[i].first;
            marginal += pmarginal;
        }
    }
};


struct ppcbt2_t : public PF_t<particle_pcbt_t> {
    string algorithm_;
    dai::PropertySet opts_;

    ppcbt2_t(const string &name, const cellmap_t &cellmap, int nparticles) : PF_t(name, cellmap, nparticles) {
        create_particles();
    }
    virtual ~ppcbt2_t() { }

    void set_algorithm_and_options(const string &algorithm, const dai::PropertySet &opts) {
        algorithm_ = algorithm;
        opts_ = opts;
    }

    void create_particles() {
        particles_.reserve(nparticles_);
        for( int i = 0; i < nparticles_; ++i )
            particles_.push_back(make_pair(1, particle_pcbt_t(&cellmap_)));
    }

    virtual void initialize(int initial_loc) {
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1 / float(nparticles_);
            particles_[i].second.reset_factors(initial_loc);
        }
    }

    virtual void update(int last_action, int obs) {
        vector<int> indices;
        PF_t::stochastic_universal_sampling(nparticles_, indices);
        assert(indices.size() == size_t(nparticles_));

        float total_mass = 0.0;
        vector<pair<float, particle_pcbt_t> > new_particles;
        new_particles.reserve(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            int index = indices[i];
            const particle_pcbt_t &p = particles_[index].second;
            int new_loc = p.new_loc(last_action, obs);

            float new_weight = 0;
            for( int label = 0; label < nlabels_; ++label ) { // marginalize over possible labels at current loc
                new_weight += cellmap_.obs_probability(obs, label, last_action) * p.probability(label, new_loc);
            }

            particle_pcbt_t np = p;
            np.update(last_action, obs, new_loc);
            new_particles.push_back(make_pair(new_weight, np));
            total_mass += new_weight;
        }
        for( int i = 0; i < nparticles_; ++i )
            new_particles[i].first /= total_mass;
        particles_ = new_particles;
    }

    virtual void calculate_marginals() {
        for( size_t i = 0; i < particles_.size(); ++i )
            particles_[i].second.calculate_marginals(algorithm_, opts_);
    }

    virtual void get_marginal(int var, marginal_t &marginal) const {
        marginal_t pmarginal;
        marginal = marginal_t(var < nloc_ ? nlabels_ : nloc_);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].second.get_marginal(var, pmarginal);
            pmarginal *= particles_[i].first;
            marginal += pmarginal;
        }
    }
};



void usage(ostream &os) {
    os << endl
       << "Usage: maplearning [{-t | --ntrials} <ntrials>]" << endl
       << "                   [{-r | --nrows} <nrows>]" << endl
       << "                   [{-c | --ncols} <ncols>]" << endl
       << "                   [{-l | --nlabels} <nlabels>]" << endl
       << "                   [{-s | --seed} <seed>]" << endl
       << "                   [{-v | --verbose}]" << endl
       << "                   [{-p | --policy} <policy>]" << endl
       << "                   [{-? | --help}]" << endl
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
    int nrows = 1;
    int ncols = 8;
    int nlabels = 2;
    int nsteps = 0;
    int seed = 0;
    bool verbose = false;
    vector<string> tracking_algorithms_str;

    float pa = 0.8;
    float po = 0.9;

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
        } else if( !strcmp(argv[0], "-l") || !strcmp(argv[0], "--nlabels") ) {
            nlabels = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-n") || !strcmp(argv[0], "--nsteps") ) {
            nsteps = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "--pa") ) {
            pa = strtod(argv[1], 0);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "--po") ) {
            po = strtod(argv[1], 0);
            argc -= 2;
            argv += 2;
        } else if( !strncmp(argv[0], "--tracking=", 11) ) {
            char *str = strdup(&argv[0][11]);
            char *token = strtok(str, ",");
            while( token != 0 ) {
                tracking_algorithms_str.push_back(token);
                token = strtok(0, ",");
            }
            free(str);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-v") || !strcmp(argv[0], "--verbose") ) {
            verbose = true;
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-p") || !strcmp(argv[0], "--policy") ) {
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-?") || !strcmp(argv[0], "--help") ) {
            usage(cerr);
            exit(-1);
        } else {
            cerr << "error: unexpected argument: " << argv[0] << endl;
            --argc;
            ++argv;
        }
    }

    // set seed
    unsigned short seeds[3];
    seeds[0] = seeds[1] = seeds[2] = seed;
    seed48(seeds);

    // set static members
    coord_t::ncols_ = ncols;

    // create cellmap and tracking algorithms
    cellmap_t cellmap(nrows, ncols, nlabels, pa, po, 0.0);

    // tracking algorithms
    vector<tracking_t*> tracking_algorithms;
    dai::PropertySet opts;
    int nparticles = 100;
    int memory = 0;

    for( size_t i = 0; i < tracking_algorithms_str.size(); ++i ) {
        char *str = strdup(tracking_algorithms_str[i].c_str());
        string name = strtok(str, ":");
        char *token = strtok(0, ":");
        if( name == "jt" ) {
            memory = token != 0 ? atoi(token) : memory;
            pcbt_t *pcbt = new pcbt_t(name + "_" + to_string(memory), cellmap, memory);
            pcbt->set_algorithm_and_options("JT", opts("updates", string("HUGIN")));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "bp" ) {
            memory = token != 0 ? atoi(token) : memory;
            pcbt_t *pcbt = new pcbt_t(name + "_" + to_string(memory), cellmap, memory);
            pcbt->set_algorithm_and_options("BP", opts("updates", string("SEQRND"))("logdomain", false)("tol", 1e-9)("maxiter", (size_t)10000));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "hak" ) {
            memory = token != 0 ? atoi(token) : memory;
            pcbt_t *pcbt = new pcbt_t(name + "_" + to_string(memory), cellmap, memory);
            pcbt->set_algorithm_and_options("HAK", opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "ppcbt_jt" ) {
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 1000);
            pcbt->set_algorithm_and_options("JT", opts("updates", string("HUGIN")));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "ppcbt_bp" ) {
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 50);
            pcbt->set_algorithm_and_options("HAK", opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "ppcbt_hak" ) {
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 50);
            pcbt->set_algorithm_and_options("BP", opts("updates", string("SEQRND"))("logdomain", false)("tol", 1e-9)("maxiter", (size_t)10000));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "sis" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new SIS_t(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "mm_sir" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new motion_model_SIR_t(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "mm_sir2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new motion_model_SIR2_t(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "opt_sir" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new optimal_SIR_t(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "opt_sir2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new optimal_SIR2_t(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "rbpf2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new RBPF2_t(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else {
            cerr << "warning: unrecognized tracking algorithm '" << name << "'" << endl;
        }
        free(str);
    }

    // compute longest name
    size_t size_longest_name = 0;
    for( size_t i = 0; i < tracking_algorithms.size(); ++i )
        size_longest_name = size_longest_name > tracking_algorithms[i]->name_.size() ? size_longest_name : tracking_algorithms[i]->name_.size();

    // action selection
    naive_action_selection_t policy(cellmap);

    int labels[] = { 0, 1, 0, 1, 0, 1, 0, 1 };

    cellmap_t::execution_t fixed_execution;
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, -1));
    fixed_execution.push_back(cellmap_t::execution_step_t(1, 1, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(2, 0, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(4, 0, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(5, 1, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(6, 0, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(5, 1, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(4, 0, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(2, 0, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(1, 1, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, cellmap_t::left));

    // run for the specified number of trials
    for( int trial = 0; trial < ntrials; ++trial ) {
        cellmap_t::execution_t output_execution;
        if( (nsteps == 0) && (nrows == 1) && (ncols == 8) )
            cellmap.run_execution(labels, 8, fixed_execution, output_execution, tracking_algorithms);
        else
            cellmap.run_execution(labels, 0, output_execution, nsteps, policy, tracking_algorithms);

        // calculate final marginals
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            tracking_algorithms[i]->calculate_marginals();

        // compute and print scores
        vector<cellmap_t::score_t> scores;
        cellmap.compute_scores(output_execution, tracking_algorithms, scores);
        for( size_t i = 0; i < scores.size(); ++i ) {
            cout << "# score(" << setw(size_longest_name) << tracking_algorithms[i]->name_ << "): score=" << scores[i] << endl;
        }

        // print final (tracked) map and location
        cout << "# final(" << setw(size_longest_name) << "real" << "): map=[";
        for( int var = 0; var < nrows * ncols; ++var )
            cout << " " << cellmap.cells_[var].label_;
        cout << "], loc=" << output_execution.back().loc_ << endl;
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            cout << "# final(" << setw(size_longest_name) << tracking_algorithms[i]->name_ << "): map=[";
            for( int var = 0; var < nrows * ncols; ++var )
                cout << " " << tracking_algorithms[i]->MAP_on_var(var);
            cout << "], loc=" << tracking_algorithms[i]->MAP_on_var(nrows * ncols) << endl;
        }

        // generate R plots
        cout << "library(\"reshape2\");" << endl << "library(\"ggplot2\");" << endl;
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            cellmap.generate_R_plot(cout, *tracking_algorithms[i]);
    }

    return 0;
}




#if 0
            // compute and print exact marginals via junction tree
            marginal_t P1, Q1;
            //cout << "Marginals (exact: junction tree):" << endl;
            apply_junction_tree(P1, Q1);
            store_marginal(nrows_ * ncols_, &cellmap_t::jt_marginal, marginals);
            //print_marginals(cout, &cellmap_t::jt_marginal, 10);

            // approximate and print marginals
            marginal_t P2, Q2;
            //cout << "Marginals (approx. inference):" << endl;
            apply_approx_inference(P2, Q2);
            //print_marginals(cout, &cellmap_t::approx_inference_marginal);

            // calculate KL-divergence
            pair<float, bool> js1 = JS_divergence(P1, P2);
            pair<float, bool> js2 = JS_divergence(Q1, Q2);
            cout << "JS-divergence: P[jt,approx]=" << js1.first << ", Q[jt,approx]=" << js2.first << endl;

    void apply_approx_inference(marginal_t &P, marginal_t &Q) const {
        apply_approx_inference();
        P.clear();
        float mass = 0;
        for( int i = 0; i < nloc_; ++i ) {
            P.push_back(approx_inference_algorithm_->belief(approx_inference_fg_.var(i))[0]);
            mass = P.back();
        }
        for( int i = 0; i < nloc_; ++i ) P[i] /= mass;
        Q.clear();
        for( int i = 0; i < nloc_; ++i )
            Q.push_back(approx_inference_algorithm_->belief(approx_inference_fg_.var(nloc_))[i]);
    }

    // inference algorithms for MAP
    void apply_junction_tree_for_MAP(vector<size_t> &state) const {
        dai::PropertySet opts;
        jt_fg_ = dai::FactorGraph(factors_);
        jt_ = dai::JTree(jt_fg_, opts("updates", string("HUGIN"))("inference", string("MAXPROD")));
        jt_.init();
        jt_.run();
        state = jt_.findMaximum();
    }

        // Calculate MAP over cellmap labels
        vector<size_t> map_value;
        cellmap.apply_junction_tree_for_MAP(map_value);
        for( size_t i = 0; i < map_value.size(); ++i ) {
            cout << "v" << i << " = " << map_value[i];
            if( int(i) < nrows * ncols ) cout << " [label=" << cellmap.cells_[i].label_ << "]";
            cout << endl;
        }
#endif

