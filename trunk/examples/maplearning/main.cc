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

// forward references
struct cellmap_t;

inline ostream& operator<<(ostream &os, const vector<float> &dist) {
    //os << "[";
    for( size_t i = 0; i < dist.size(); ++i )
        //os << " " << i << "->" << dist[i];
        os << " " << dist[i] << (i + 1 < dist.size() ? "," : "");
    //os << " ]";
    return os;
}

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

struct tracking_t {
    string name_;
    const cellmap_t &cellmap_;
    int nloc_;
    int nlabels_;

    vector<vector<vector<float> > > marginals_;

    tracking_t(const string &name, const cellmap_t &cellmap);
    virtual void initialize(int initial_loc) = 0;
    virtual void update(int last_action, int obs) = 0;
    virtual void calculate_marginals() = 0;
    virtual void get_marginal(int var, vector<float> &marginal) const = 0;

    void store_marginals() {
        marginals_.push_back(vector<vector<float> >(nloc_ + 1));
        for( int var = 0; var < nloc_; ++var )
            get_marginal(var, marginals_.back()[var]);
        get_marginal(nloc_, marginals_.back()[nloc_]);
    }

    const vector<float>& stored_marginal(int t, int var) const {
        assert(t < int(marginals_.size()));
        const vector<vector<float> > &marginals_at_time_t = marginals_[t];
        assert(var < int(marginals_at_time_t.size()));
        return marginals_at_time_t[var];
    }

    int MAP_on_var(int var) const {
        vector<float> marginal;
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

struct action_selection_t {
    const cellmap_t &cellmap_;
    action_selection_t(const cellmap_t &cellmap) : cellmap_(cellmap) { }
    int select_action(const tracking_t *tracking = 0) const;
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

    float obs_probability(int obs, int label, int /*last_action*/) const {
        return label == obs ? po_ : (1 - po_) / float(nlabels_ - 1);
    }
    float obs_probability(int obs, int loc, const vector<int> &labels, int last_action) const {
        return obs_probability(obs, labels[loc], last_action);
    }

    // run execution: action application and observation sampling
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

    pair<int, int> run_execution(const int *labels, int labels_size, const execution_t &execution, int nsteps, const action_selection_t *policy, vector<tracking_t*> &tracking_algorithms) {
        // set labels for callmap
        if( labels_size > 0 )
            set_labels(labels, labels_size);
        else
            sample_labels();

        // initialize tracking algorithms
        initialize(0, tracking_algorithms);

        int hidden_loc = 0;
        for( size_t t = 0; true; ++t ) {
            if( !execution.empty() && (t >= execution.size()) ) return make_pair(t, hidden_loc);
            if( execution.empty() && (t >= size_t(nsteps)) ) return make_pair(t, hidden_loc);
            int last_action = -1;
            int obs = -1;

            // select next action, update hidden loc, and sample observation
            if( !execution.empty() ) {
                hidden_loc = execution[t].loc_;
                obs = execution[t].obs_;
                last_action = execution[t].last_action_;
            } else {
                assert(!tracking_algorithms.empty());
                last_action = policy->select_action(tracking_algorithms[0]);
                hidden_loc = sample_loc(hidden_loc, last_action);
                obs = sample_obs(hidden_loc, last_action);
                cout << "step (t=" << t << "): last_action=" << last_action << ", obs=" << obs << ", loc=" << hidden_loc << endl;
            } 

            // update tracking
            advance_step(last_action, obs, tracking_algorithms);
        }
    }
    pair<int, int> run_execution(const int *labels, int labels_size, const execution_t &execution, vector<tracking_t*> &tracking_algorithms) {
        return run_execution(labels, labels_size, execution, 0, 0, tracking_algorithms);
    }
    pair<int, int> run_execution(const int *labels, int labels_size, int nsteps, const action_selection_t &policy, vector<tracking_t*> &tracking_algorithms) {
        execution_t empty_execution;
        return run_execution(labels, labels_size, empty_execution, nsteps, &policy, tracking_algorithms);
    }
    pair<int, int> run_execution(const vector<int> &labels, const execution_t &execution, vector<tracking_t*> &tracking_algorithms) {
        return run_execution(&labels[0], labels.size(), execution, 0, 0, tracking_algorithms);
    }
    pair<int, int> run_execution(const vector<int> &labels, int nsteps, const action_selection_t &policy, vector<tracking_t*> &tracking_algorithms) {
        execution_t empty_execution;
        return run_execution(&labels[0], labels.size(), empty_execution, nsteps, &policy, tracking_algorithms);
    }

    // scoring of tracking algorithms
    typedef int score_t;

    score_t compute_score(int current_loc, const tracking_t &tracking) const {
        score_t score = 0;
        for( int var = 0; var < 1 + nrows_ * ncols_; ++var ) {
            vector<float> marginal;
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
        return score;
    }

    void compute_scores(int current_loc, const vector<tracking_t*> &tracking_algorithms, vector<score_t> &scores) const {
        scores.clear();
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            scores.push_back(compute_score(current_loc, *tracking_algorithms[i]));
    }

    // R plots
    void generate_R_plot(ostream &os, const tracking_t &tracking) const {
        for( size_t t = 0; t < tracking.marginals_.size(); ++t ) {
            os << "mar_" << tracking.name_ << "_t" << t << " = c(" << tracking.stored_marginal(t, nrows_ * ncols_) << ")" << endl;
        }
        os << "mar_" << tracking.name_ << " = c(";
        for( size_t t = 0; t < tracking.marginals_.size(); ++t )
            os << "mar_" << tracking.name_ << "_t" << t << (t + 1 < tracking.marginals_.size() ? ", " : "");
        os << ")" << endl;

        os << "dmf=melt(as.data.frame(t(matrix(mar_" << tracking.name_ << ", ncol=" << ncols_ << ", byrow=T))))" << endl
           << "dmf$y = rep(1:" << ncols_ << ", " << tracking.marginals_.size() << ")" << endl
           << "p <- ggplot(dmf,aes(y=variable, x=y)) + "
           << "geom_tile(aes(fill=value)) + "
           << "geom_text(aes(label=ifelse(value>.01, trunc(100*value, digits=2)/100, \"\")), size=3, angle=0, color=\"white\") +"
           << "labs(y=\"time step\", x=\"location\") + "
           << "theme_minimal() + "
           << "scale_x_discrete(limits=1:" << ncols_ << ") + "
           << "scale_y_discrete(labels=paste(\"t=\", 0:" << tracking.marginals_.size() << ", sep=\"\"))" << endl;

        os << "pdf(\"marginals_" << tracking.name_ << ".pdf\")" << endl
           << "p" << endl
           << "dev.off()" << endl;
    }
};

tracking_t::tracking_t(const string &name, const cellmap_t &cellmap) : name_(name), cellmap_(cellmap) {
    nloc_ = cellmap_.nrows_ * cellmap_.ncols_;
    nlabels_ = cellmap_.nlabels_;
}

int action_selection_t::select_action(const tracking_t *tracking) const {
    int action = drand48() > .30 ? cellmap_t::right : cellmap_t::left;
    return action;
}


struct pcbt_t : public tracking_t {
    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;

    mutable dai::FactorGraph fg_;
    mutable dai::InfAlg *inference_algorithm_;

    pcbt_t(const string &name, const cellmap_t &cellmap) : tracking_t(name, cellmap) {
        create_variables_and_factors();
    }

    void create_variables_and_factors() {
        variables_ = vector<dai::Var>(cellmap_.nrows_ * cellmap_.ncols_ + 1);
        for( int i = 0; i < nloc_; ++i )
            variables_[i] = dai::Var(i, nlabels_);
        variables_[nloc_] = dai::Var(nloc_, nloc_);
        factors_ = vector<dai::Factor>(nloc_);
    }

    void reset_factors(size_t initial_loc) {
        float p = 1.0 / float(nlabels_);
        for( int i = 0; i < nloc_; ++i ) {
            factors_[i] = dai::Factor(dai::VarSet(variables_[i], variables_[nloc_]), 0.0);
            for( size_t j = 0; j < factors_[i].nrStates(); ++j ) {
                if( j / nlabels_ == initial_loc ) factors_[i].set(j, p);
            }
        }
    }

    void filter_with_obs(int floc, dai::Factor &factor, int /*last_action*/, int obs) const {
        //cout << "FILTER[obs=" << obs << "]: old factor[" << floc << "]: " << factor << endl;
        float po = cellmap_.po_;
        for( int j = 0; j < nlabels_ * nloc_; ++j ) {
            float weight = factor[j];
            int label = j % nlabels_;
            int loc = j / nlabels_;
            if( floc == loc )
                factor.set(j, weight * (label == obs ? po : (1 - po) / float(nlabels_ - 1)));
            else
                factor.set(j, weight * 0.5);
        }
        //cout << "FILTER[obs=" << obs << "]: new factor[" << floc << "]: " << factor << endl;
    }

    void update_factor(int i, int last_action, int obs) {
        //cout << "UPDATE[last_action=" << last_action << ", obs=" << obs << "]: old factor[" << i << "]: " << factors_[i] << endl;
        if( last_action != -1 ) {
            dai::Factor &factor = factors_[i];
            dai::Factor new_factor(factor.vars(), 0.0);
            for( size_t j = 0; j < factor.nrStates(); ++j ) {
                int new_label = j % nlabels_;
                int new_loc = j / nlabels_;
                float new_weight = 0.0;
                for( int loc = 0; loc < nloc_; ++loc ) {
                    float weight = factor[new_label + nlabels_ * loc];
                    new_weight += cellmap_.loc_probability(last_action, loc, new_loc) * weight;
                }
                new_factor.set(j, new_weight);
            }
            factors_[i] = new_factor;
        }
        if( obs != -1 ) {
            filter_with_obs(i, factors_[i], last_action, obs);
        }
        //cout << "UPDATE[last_action=" << last_action << ", obs=" << obs << "]: new factor[" << i << "]: " << factors_[i] << endl;
    }

    virtual void initialize(int initial_loc) {
        marginals_.clear();
        reset_factors(initial_loc);
    }

    virtual void update(int last_action, int obs) {
        assert((obs >= 0) && (obs < nlabels_));
        for( int i = 0; i < nloc_; ++i )
            update_factor(i, last_action, obs);
    }

    virtual void get_marginal(int var, vector<float> &marginal) const {
        const dai::Factor &factor = inference_algorithm_->belief(fg_.var(var));
        marginal = vector<float>(factor.nrStates());
        for( size_t i = 0; i < factor.nrStates(); ++i )
            marginal[i] = factor[i];
    }
};

struct pcbt_jt_t : public pcbt_t {
    const dai::PropertySet opts_;

    pcbt_jt_t(const string &name, const cellmap_t &cellmap, const dai::PropertySet &opts) : pcbt_t(name, cellmap), opts_(opts) { }

    virtual void calculate_marginals() {
        fg_ = dai::FactorGraph(factors_);
        delete inference_algorithm_;
        inference_algorithm_ = new dai::JTree(fg_, opts_);
        inference_algorithm_->init();
        inference_algorithm_->run();
    }
};

struct pcbt_bp_t : public pcbt_t {
    string algorithm_;
    const dai::PropertySet opts_;

    pcbt_bp_t(const string &name, const cellmap_t &cellmap, const string &algorithm, const dai::PropertySet &opts)
      : pcbt_t(name, cellmap), algorithm_(algorithm), opts_(opts) { }

    virtual void calculate_marginals() {
        fg_ = dai::FactorGraph(factors_);
        delete inference_algorithm_;
        if( algorithm_ == "BP" )
            inference_algorithm_ = new dai::BP(fg_, opts_);
        else if( algorithm_ == "HAK" )
            inference_algorithm_ = new dai::HAK(fg_, opts_);
        inference_algorithm_->init();
        inference_algorithm_->run();
    }
};

// Generic Particle Filter
template <typename T> struct PF_t : public tracking_t {
    int nparticles_;
    vector<pair<float, T> > particles_;
    vector<vector<float> > marginals_on_vars_;

    PF_t(const string &name, const cellmap_t &cellmap, int nparticles) : tracking_t(name, cellmap), nparticles_(nparticles) { }
    virtual ~PF_t() { }

    virtual void initialize(int initial_loc) = 0;
    virtual void update(int last_action, int obs) = 0;
    virtual void calculate_marginals() = 0;
    virtual void get_marginal(int var, vector<float> &marginal) const = 0;

    void stochastic_universal_sampling(int n, vector<int> &indices) const {
        vector<float> cdf(nparticles_, 0);
        cdf[0] = particles_[0].first;
        for( int i = 1; i < nparticles_; ++i )
            cdf[i] = cdf[i - 1] + particles_[i].first;

        indices.clear();
        indices.reserve(n);
        float u = drand48() / float(n);
        for( int i = 0, j = 0; j < n; ++j ) {
            assert(u <= 1.0);
            while( u > cdf[i] ) ++i;
            indices.push_back(i);
            assert(i < nparticles_);
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
    SIS_t(const string &name, const cellmap_t &cellmap, int nparticles) : PF_t(name, cellmap, nparticles) { }
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
            vector<int> &state = particles_[i].second;
            int new_loc = cellmap_.sample_loc(state[nloc_], last_action);
            weight *= cellmap_.obs_probability(obs, state[nloc_], state, last_action);
            state[nloc_] = new_loc;
        }
    }

    virtual void calculate_marginals() {
        marginals_on_vars_ = vector<vector<float> >(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            marginals_on_vars_[i] = vector<float>(nlabels_, 0.0);
        marginals_on_vars_[nloc_] = vector<float>(nloc_, 0.0);

        float total_mass = 0;
        for( int i = 0; i < nparticles_; ++i )
            total_mass += particles_[i].first;

        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const vector<int> &state = particles_[i].second;
            for( int j = 0; j <= nloc_; ++j )
                marginals_on_vars_[j][state[j]] += weight / total_mass;
        }
    }

    virtual void get_marginal(int var, vector<float> &marginal) const {
        marginal = marginals_on_vars_[var];
    }
};

// Generic Sequential Importance Resampling (SIR) Particle Filter
struct Gen_SIR_t : public PF_t<vector<int> > {
    vector<pair<int, int> > history_;

    Gen_SIR_t(const string &name, const cellmap_t &cellmap, int nparticles) : PF_t(name, cellmap, nparticles) { }
    virtual ~Gen_SIR_t() { }

    virtual void sample_from_pi(vector<int> &new_state, const vector<int> &state, int last_action, int obs) const = 0;
    //virtual float pi(const vector<int> &new_state, const vector<int> &state, int last_action, int obs) const = 0;
    virtual float importance_weight(const vector<int> &new_state, const vector<int> &state, int last_action, int obs) const = 0;

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
            vector<int> &state = particles_[index].second;
            vector<int> new_state;
            sample_from_pi(new_state, state, last_action, obs);
            float weight = importance_weight(new_state, state, last_action, obs);
            total_mass += weight;
            new_particles.push_back(make_pair(weight, new_state));
        }
        for( int i = 0; i < nparticles_; ++i )
            new_particles[i].first /= total_mass;
        particles_ = new_particles;

        history_.push_back(make_pair(last_action, obs));
    }

    virtual void calculate_marginals() {
        marginals_on_vars_ = vector<vector<float> >(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            marginals_on_vars_[i] = vector<float>(nlabels_, 0.0);
        marginals_on_vars_[nloc_] = vector<float>(nloc_, 0.0);

        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const vector<int> &state = particles_[i].second;
            for( int j = 0; j < nloc_; ++j )
                marginals_on_vars_[j][state[j]] += weight;
            marginals_on_vars_[nloc_][state.back()] += weight;
        }
    }

    virtual void get_marginal(int var, vector<float> &marginal) const {
        marginal = marginals_on_vars_[var];
    }
};

struct Optimal_SIR_t : public Gen_SIR_t {
    int nstates_;
    float **cdf_;

    Optimal_SIR_t(const string &name, const cellmap_t &cellmap, int nparticles) : Gen_SIR_t(name, cellmap, nparticles) {
        nstates_ = nloc_;
        for( int loc = 0; loc < nloc_; ++loc )
            nstates_ *= nlabels_;
        calculate_cdf_for_pi();
    }
    virtual ~Optimal_SIR_t() {
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
            //cout << "cdf[" << i << "/" << nstates_ * nlabels_ * 4 << "][" << nstates_ - 1 << "]=" << setprecision(9) << cdf_[i][nstates_ - 1] << endl;
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
        //
        // since P(new_state|state,last_action) = 0 if state and new_states have different labels,
        // we can simplify the sum over locations instead of states
        float p = 0;
        int loc = state[nloc_];
        for( int new_loc = 0; new_loc < nloc_; ++new_loc )
            p += cellmap_.obs_probability(obs, new_loc, state, last_action) * cellmap_.loc_probability(last_action, loc, new_loc);
        return p;
    }
};

struct Motion_Model_SIR_t : public Gen_SIR_t {
    Motion_Model_SIR_t(const string &name, const cellmap_t &cellmap, int nparticles) : Gen_SIR_t(name, cellmap, nparticles) { }
    virtual ~Motion_Model_SIR_t() { }

    virtual void sample_from_pi(vector<int> &new_state, const vector<int> &state, int last_action, int /*obs*/) const {
        new_state = state;
        int new_loc = cellmap_.sample_loc(state[nloc_], last_action);
        new_state[nloc_] = new_loc;
    }
#if 0
    virtual float pi(const vector<int> &new_state, const vector<int> &state, int last_action, int /*obs*/) const {
        for( int loc = 0; loc < nloc_; ++loc )
            if( new_state[loc] != state[loc] ) return 0;
        return cellmap_.loc_probability(last_action, state[nloc_], new_state[nloc_]);
    }
#endif
    virtual float importance_weight(const vector<int> &/*new_state*/, const vector<int> &state, int last_action, int obs) const {
        return cellmap_.obs_probability(obs, state[nloc_], state, last_action);
    }
};

struct RBPF_t : public Gen_SIR_t {
    RBPF_t(const string &name, const cellmap_t &cellmap, int nparticles) : Gen_SIR_t(name, cellmap, nparticles) { }
    virtual ~RBPF_t() { }

    virtual void sample_from_pi(vector<int> &new_state, const vector<int> &state, int last_action, int /*obs*/) const {
        new_state = state;
        int new_loc = cellmap_.sample_loc(state.back(), last_action);
        new_state.push_back(new_loc);
    }
#if 0
    virtual float pi(const vector<int> &new_state, const vector<int> &state, int last_action, int /*obs*/) const {
        for( int loc = 0; loc < nloc_; ++loc )
            if( new_state[loc] != state[loc] ) return 0;
        return cellmap_.loc_probability(last_action, state.back(), new_state.back());
    }
#endif
    virtual float importance_weight(const vector<int> &new_state, const vector<int> &state, int last_action, int obs) const {
        float p = 0;
        for( int label = 0; label < nlabels_; ++label )
            p += cellmap_.obs_probability(obs, label, last_action) * probability(new_state.back(), label, state);
        return p;
    }

    float probability(int current_loc, int label, const vector<int> &state) const {
        if( !history_.empty() ) {
            float freq_at_current_loc = 0;
            float freq_same_label_at_current_loc = 0;
            for( size_t i = nloc_ + 1; i < state.size(); ++i ) {
                int loc = state[i];
                if( loc == current_loc ) {
                    ++freq_at_current_loc;
                    assert(history_.size() > i - nloc_ - 1);
                    freq_same_label_at_current_loc = label == history_[i - nloc_ - 1].second ? 1 : 0;
                }
            }
            return freq_at_current_loc == 0 ? 1 / float(nlabels_) : float(freq_same_label_at_current_loc) / float(freq_at_current_loc);
        } else {
            return 1 / float(nlabels_);
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
        } else if( !strcmp(argv[0], "-v") || !strcmp(argv[0], "--verbose") ) {
            verbose = true;
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-p") || !strcmp(argv[0], "--policy") ) {
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

    // set static members
    coord_t::ncols_ = ncols;

    // create cellmap and tracking algorithms
    cellmap_t cellmap(nrows, ncols, nlabels, pa, po, 0.0);

    // tracking algorithms
    vector<tracking_t*> tracking_algorithms;
    dai::PropertySet opts;

    pcbt_jt_t jt("jt", cellmap, opts("updates", string("HUGIN")));
    pcbt_bp_t bp("bp", cellmap, "BP", opts("updates", string("SEQRND"))("logdomain", false)("tol", 1e-9)("maxiter", (size_t)10000));
    pcbt_bp_t hak("hak", cellmap, "HAK", opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));

    int nparticles = 250;
    SIS_t sis("sis", cellmap, nparticles);
    Motion_Model_SIR_t mm_sir("mm_sir", cellmap, nparticles);
    Optimal_SIR_t opt_sir("opt_sir", cellmap, nparticles);
    RBPF_t rbpf("rbpf", cellmap, nparticles);

    tracking_algorithms.push_back(&jt);
    tracking_algorithms.push_back(&bp);
    tracking_algorithms.push_back(&hak);
    tracking_algorithms.push_back(&sis);
    tracking_algorithms.push_back(&mm_sir);
    tracking_algorithms.push_back(&opt_sir);
    tracking_algorithms.push_back(&rbpf);

    // action selection
    action_selection_t policy(cellmap);

    int labels[] = { 0, 1, 0, 1, 0, 1, 0, 1 };

    cellmap_t::execution_t fixed_execution;
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
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 1, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, cellmap_t::left));

    // run for the specified number of trials
    cout << "initialization completed!" << endl;
    for( int trial = 0; trial < ntrials; ++trial ) {
        pair<int, int> p;
        if( (nsteps == 0) && (nrows == 1) && (ncols == 8) )
            p = cellmap.run_execution(labels, 8, fixed_execution, tracking_algorithms);
        else
            p = cellmap.run_execution(labels, 0, nsteps, policy, tracking_algorithms);

        // calculate final marginals
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            tracking_algorithms[i]->calculate_marginals();

        // compute and print scores
        vector<cellmap_t::score_t> scores;
        cellmap.compute_scores(0, tracking_algorithms, scores);
        for( size_t i = 0; i < scores.size(); ++i ) {
            cout << "score(" << setw(6) << tracking_algorithms[i]->name_ << "): score=" << scores[i] << endl;
        }

        // print final (tracked) map and location
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            cout << "final(" << setw(6) << tracking_algorithms[i]->name_ << "): map=[";
            for( int var = 0; var < nrows * ncols; ++var )
                cout << " " << tracking_algorithms[i]->MAP_on_var(var);
            cout << "], loc=" << tracking_algorithms[i]->MAP_on_var(nrows * ncols) << endl;
        }

        // generate R plots
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            cellmap.generate_R_plot(cout, *tracking_algorithms[i]);
    }

    return 0;
}




#if 0
            // compute and print exact marginals via junction tree
            vector<float> P1, Q1;
            //cout << "Marginals (exact: junction tree):" << endl;
            apply_junction_tree(P1, Q1);
            store_marginal(nrows_ * ncols_, &cellmap_t::jt_marginal, marginals);
            //print_marginals(cout, &cellmap_t::jt_marginal, 10);

            // approximate and print marginals
            vector<float> P2, Q2;
            //cout << "Marginals (approx. inference):" << endl;
            apply_approx_inference(P2, Q2);
            //print_marginals(cout, &cellmap_t::approx_inference_marginal);

            // calculate KL-divergence
            pair<float, bool> js1 = JS_divergence(P1, P2);
            pair<float, bool> js2 = JS_divergence(Q1, Q2);
            cout << "JS-divergence: P[jt,approx]=" << js1.first << ", Q[jt,approx]=" << js2.first << endl;
#endif
 
#if 0
    void apply_approx_inference(vector<float> &P, vector<float> &Q) const {
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
#endif

#if 0
    // inference algorithms for MAP
    void apply_junction_tree_for_MAP(vector<size_t> &state) const {
        dai::PropertySet opts;
        jt_fg_ = dai::FactorGraph(factors_);
        jt_ = dai::JTree(jt_fg_, opts("updates", string("HUGIN"))("inference", string("MAXPROD")));
        jt_.init();
        jt_.run();
        state = jt_.findMaximum();
    }
#endif

#if 0
        // Calculate MAP over cellmap labels
        vector<size_t> map_value;
        cellmap.apply_junction_tree_for_MAP(map_value);
        for( size_t i = 0; i < map_value.size(); ++i ) {
            cout << "v" << i << " = " << map_value[i];
            if( int(i) < nrows * ncols ) cout << " [label=" << cellmap.cells_[i].label_ << "]";
            cout << endl;
        }
#endif

