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
    const cellmap_t &cellmap_;
    int nloc_;
    int nlabels_;

    vector<vector<vector<float> > > marginals_;

    tracking_t(const cellmap_t &cellmap);
    virtual void initialize(int initial_pos) = 0;
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
    coord_t(int pos) : col_(pos % ncols_), row_(pos / ncols_) { }
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

    float probability(int action, int pos, int new_pos) const {
        float p = 0;
        coord_t coord(pos), new_coord(new_pos);
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

    float obs_probability(int obs, int pos, int /*last_action*/) const {
        return cells_[pos].label_ == obs ? po_ : (1 - po_) / float(nlabels_ - 1);
    }

    // run execution: action application and observation sampling
    int sample_location(int pos, int action) const {
        coord_t coord(pos), new_coord(pos);
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

    int sample_obs(int pos, int /*last_action*/) const {
        assert((pos >= 0) && (pos < int(cells_.size())));
        int label = cells_[pos].label_;
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

    void initialize(int initial_pos, vector<pair<string, tracking_t*> > &tracking_algorithms) const {
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            tracking_t &tracking = *tracking_algorithms[i].second;
            tracking.initialize(initial_pos);
            tracking.calculate_marginals();
            tracking.store_marginals();
        }
    }
    void advance_step(int last_action, int obs, vector<pair<string, tracking_t*> > &tracking_algorithms) const {
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            tracking_t &tracking = *tracking_algorithms[i].second;
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

    pair<int, int> run_execution(const int *labels, int labels_size, const execution_t &execution, int nsteps, const action_selection_t *policy, vector<pair<string, tracking_t*> > &tracking_algorithms) {
        // set labels for callmap
        if( labels_size > 0 )
            set_labels(labels, labels_size);
        else
            sample_labels();

        // initialize tracking algorithms
        initialize(0, tracking_algorithms);

        int hidden_pos = 0;
        for( size_t t = 0; true; ++t ) {
            if( !execution.empty() && (t >= execution.size()) ) return make_pair(t, hidden_pos);
            if( execution.empty() && (t >= size_t(nsteps)) ) return make_pair(t, hidden_pos);
            int last_action = -1;
            int obs = -1;

            // select next action, update hidden pos, and sample observation
            if( !execution.empty() ) {
                hidden_pos = execution[t].loc_;
                obs = execution[t].obs_;
                last_action = execution[t].last_action_;
            } else {
                assert(!tracking_algorithms.empty());
                last_action = policy->select_action(tracking_algorithms[0].second);
                hidden_pos = sample_location(hidden_pos, last_action);
                obs = sample_obs(hidden_pos, last_action);
                cout << "step (t=" << t << "): last_action=" << last_action << ", obs=" << obs << ", pos=" << hidden_pos << endl;
            } 

            // update tracking
            advance_step(last_action, obs, tracking_algorithms);
        }
    }
    pair<int, int> run_execution(const int *labels, int labels_size, const execution_t &execution, vector<pair<string, tracking_t*> > &tracking_algorithms) {
        return run_execution(labels, labels_size, execution, 0, 0, tracking_algorithms);
    }
    pair<int, int> run_execution(const int *labels, int labels_size, int nsteps, const action_selection_t &policy, vector<pair<string, tracking_t*> > &tracking_algorithms) {
        execution_t empty_execution;
        return run_execution(labels, labels_size, empty_execution, nsteps, &policy, tracking_algorithms);
    }
    pair<int, int> run_execution(const vector<int> &labels, const execution_t &execution, vector<pair<string, tracking_t*> > &tracking_algorithms) {
        return run_execution(&labels[0], labels.size(), execution, 0, 0, tracking_algorithms);
    }
    pair<int, int> run_execution(const vector<int> &labels, int nsteps, const action_selection_t &policy, vector<pair<string, tracking_t*> > &tracking_algorithms) {
        execution_t empty_execution;
        return run_execution(&labels[0], labels.size(), empty_execution, nsteps, &policy, tracking_algorithms);
    }

    // scoring of tracking algorithms
    typedef int score_t;

    score_t compute_score(int current_pos, const tracking_t &tracking) const {
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
                    score += current_pos == most_probable[0] ? 1 : 0;
                }
            }
        }
        return score;
    }

    void compute_scores(int current_pos, const vector<pair<string, tracking_t*> > &tracking_algorithms, vector<score_t> &scores) const {
        scores.clear();
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            scores.push_back(compute_score(current_pos, *tracking_algorithms[i].second));
    }

    // R plots
    void generate_R_plot(ostream &os, const tracking_t &tracking, const string &name) const {
        for( size_t t = 0; t < tracking.marginals_.size(); ++t ) {
            os << "mar_" << name << "_t" << t << " = c(" << tracking.stored_marginal(t, nrows_ * ncols_) << ")" << endl;
        }
        os << "mar_" << name << " = c(";
        for( size_t t = 0; t < tracking.marginals_.size(); ++t )
            os << "mar_" << name << "_t" << t << (t + 1 < tracking.marginals_.size() ? ", " : "");
        os << ")" << endl;

        os << "dmf=melt(as.data.frame(t(matrix(mar_" << name << ", ncol=" << ncols_ << ", byrow=T))))" << endl
           << "dmf$y = rep(1:" << ncols_ << ", " << tracking.marginals_.size() << ")" << endl
           << "p <- ggplot(dmf,aes(y=variable, x=y)) + "
           << "geom_tile(aes(fill=value)) + "
           << "geom_text(aes(label=ifelse(value>.01, trunc(100*value, digits=2)/100, \"\")), size=3, angle=0, color=\"white\") +"
           << "labs(y=\"time step\", x=\"location\") + "
           << "theme_minimal() + "
           << "scale_x_discrete(limits=1:" << ncols_ << ") + "
           << "scale_y_discrete(labels=paste(\"t=\", 0:" << tracking.marginals_.size() << ", sep=\"\"))" << endl;

        os << "pdf(\"marginals_" << name << ".pdf\")" << endl
           << "p" << endl
           << "dev.off()" << endl;
    }
};

tracking_t::tracking_t(const cellmap_t &cellmap) : cellmap_(cellmap) {
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

    pcbt_t(const cellmap_t &cellmap) : tracking_t(cellmap) {
        create_variables_and_factors();
    }

    void create_variables_and_factors() {
        variables_ = vector<dai::Var>(cellmap_.nrows_ * cellmap_.ncols_ + 1);
        for( int i = 0; i < nloc_; ++i )
            variables_[i] = dai::Var(i, nlabels_);
        variables_[nloc_] = dai::Var(nloc_, nloc_);
        factors_ = vector<dai::Factor>(nloc_);
    }

    void reset_factors(size_t initial_pos) {
        float p = 1.0 / float(nlabels_);
        for( int i = 0; i < nloc_; ++i ) {
            factors_[i] = dai::Factor(dai::VarSet(variables_[i], variables_[nloc_]), 0.0);
            for( size_t j = 0; j < factors_[i].nrStates(); ++j ) {
                if( j / nlabels_ == initial_pos ) factors_[i].set(j, p);
            }
        }
    }

    void filter_with_obs(int fpos, dai::Factor &factor, int /*last_action*/, int obs) const {
        //cout << "FILTER[obs=" << obs << "]: old factor[" << fpos << "]: " << factor << endl;
        float po = cellmap_.po_;
        for( int j = 0; j < nlabels_ * nloc_; ++j ) {
            float weight = factor[j];
            int label = j % nlabels_;
            int pos = j / nlabels_;
            if( fpos == pos )
                factor.set(j, weight * (label == obs ? po : (1 - po) / float(nlabels_ - 1)));
            else
                factor.set(j, weight * 0.5);
        }
        //cout << "FILTER[obs=" << obs << "]: new factor[" << fpos << "]: " << factor << endl;
    }

    void update_factor(int i, int last_action, int obs) {
        //cout << "UPDATE[last_action=" << last_action << ", obs=" << obs << "]: old factor[" << i << "]: " << factors_[i] << endl;
        if( last_action != -1 ) {
            dai::Factor &factor = factors_[i];
            dai::Factor new_factor(factor.vars(), 0.0);
            for( size_t j = 0; j < factor.nrStates(); ++j ) {
                int new_label = j % nlabels_;
                int new_pos = j / nlabels_;
                float new_weight = 0.0;
                for( int pos = 0; pos < nloc_; ++pos ) {
                    float weight = factor[new_label + nlabels_ * pos];
                    new_weight += cellmap_.probability(last_action, pos, new_pos) * weight;
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

    virtual void initialize(int initial_pos) {
        marginals_.clear();
        reset_factors(initial_pos);
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

    pcbt_jt_t(const cellmap_t &cellmap, const dai::PropertySet &opts) : pcbt_t(cellmap), opts_(opts) { }

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

    pcbt_bp_t(const cellmap_t &cellmap, const string &algorithm, const dai::PropertySet &opts)
      : pcbt_t(cellmap), algorithm_(algorithm), opts_(opts) { }

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

template <typename T> struct history_t : public vector<T> { };

template <typename T> struct PF_t : public tracking_t {
    int nparticles_;
    vector<pair<float, T> > particles_;
    vector<vector<float> > marginals_on_vars_;

    PF_t(const cellmap_t &cellmap, int nparticles) : tracking_t(cellmap), nparticles_(nparticles) { }

    virtual void initialize(int initial_pos) = 0;
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
};

struct SIS_t : public PF_t<vector<int> > {
    SIS_t(const cellmap_t &cellmap, int nparticles) : PF_t(cellmap, nparticles) { }

    virtual void initialize(int initial_pos) {
        particles_ = vector<pair<float, vector<int> > >(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1;
            particles_[i].second = vector<int>(nloc_ + 1);
            for( int j = 0; j < nloc_; ++j )
                particles_[i].second[j] = lrand48() % nlabels_;
            particles_[i].second[nloc_] = initial_pos;
        }
    }

    virtual void update(int last_action, int obs) {
        for( int i = 0; i < nparticles_; ++i ) {
            float &weight = particles_[i].first;
            vector<int> &sample = particles_[i].second;
            int new_pos = cellmap_.sample_location(sample.back(), last_action);
            weight *= cellmap_.obs_probability(obs, new_pos, last_action);
            sample.back() = new_pos;
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
            const vector<int> &sample = particles_[i].second;
            for( int j = 0; j <= nloc_; ++j )
                marginals_on_vars_[j][sample[j]] += weight / total_mass;
        }
    }

    virtual void get_marginal(int var, vector<float> &marginal) const {
        marginal = marginals_on_vars_[var];
    }
};

struct SIR_t : public PF_t<vector<int> > {
    SIR_t(const cellmap_t &cellmap, int nparticles) : PF_t(cellmap, nparticles) { }

    virtual void initialize(int initial_pos) {
        particles_ = vector<pair<float, vector<int> > >(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1.0 / float(nparticles_);
            particles_[i].second = vector<int>(nloc_ + 1);
            for( int j = 0; j < nloc_; ++j )
                particles_[i].second[j] = lrand48() % nlabels_;
            particles_[i].second[nloc_] = initial_pos;
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
            vector<int> sample = particles_[index].second;
            int new_pos = cellmap_.sample_location(sample.back(), last_action);
            float weight = cellmap_.obs_probability(obs, new_pos, last_action);
            sample.back() = new_pos;
            total_mass += weight;
            new_particles.push_back(make_pair(weight, sample));
        }
        for( int i = 0; i < nparticles_; ++i )
            new_particles[i].first /= total_mass;
        particles_ = new_particles;
    }

    virtual void calculate_marginals() {
        marginals_on_vars_ = vector<vector<float> >(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            marginals_on_vars_[i] = vector<float>(nlabels_, 0.0);
        marginals_on_vars_[nloc_] = vector<float>(nloc_, 0.0);

        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const vector<int> &sample = particles_[i].second;
            for( int j = 0; j <= nloc_; ++j )
                marginals_on_vars_[j][sample[j]] += weight;
        }
    }

    virtual void get_marginal(int var, vector<float> &marginal) const {
        marginal = marginals_on_vars_[var];
    }
};

struct RBPF_t : public PF_t<vector<int> > {
    RBPF_t(const cellmap_t &cellmap, int nparticles) : PF_t(cellmap, nparticles) { }

    virtual void initialize(int initial_pos) = 0;
    virtual void update(int last_action, int obs) = 0;
    virtual void calculate_marginals() = 0;
    virtual void get_marginal(int var, vector<float> &marginal) const = 0;
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
    vector<pair<string, tracking_t*> > tracking_algorithms;
    dai::PropertySet opts;
    pcbt_jt_t jt(cellmap, opts("updates", string("HUGIN")));
    pcbt_bp_t bp(cellmap, "BP", opts("updates", string("SEQRND"))("logdomain", false)("tol", 1e-9)("maxiter", (size_t)10000));
    pcbt_bp_t hak(cellmap, "HAK", opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));
    SIS_t sis(cellmap, 50);
    SIR_t sir(cellmap, 50);
    //RBPF_t rbpf(cellmap, 50);
    tracking_algorithms.push_back(make_pair("jt", &jt));
    tracking_algorithms.push_back(make_pair("bp", &bp));
    tracking_algorithms.push_back(make_pair("hak", &hak));
    tracking_algorithms.push_back(make_pair("sis", &sis));
    tracking_algorithms.push_back(make_pair("sir", &sir));
    //tracking_algorithms.push_back(make_pair("rbpf", &rbpf));

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
    for( int trial = 0; trial < ntrials; ++trial ) {
        pair<int, int> p;
        if( (nsteps == 0) && (nrows == 1) && (ncols == 8) )
            p = cellmap.run_execution(labels, 8, fixed_execution, tracking_algorithms);
        else
            p = cellmap.run_execution(labels, 0, nsteps, policy, tracking_algorithms);

        // calculate final marginals
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            tracking_algorithms[i].second->calculate_marginals();

        // compute and print scores
        vector<cellmap_t::score_t> scores;
        cellmap.compute_scores(0, tracking_algorithms, scores);
        for( size_t i = 0; i < scores.size(); ++i ) {
            cout << "score(" << setw(4) << tracking_algorithms[i].first << "): score=" << scores[i] << endl;
        }

        // print final (tracked) map and location
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            cout << "final(" << setw(4) << tracking_algorithms[i].first << "): map=[";
            for( int var = 0; var < nrows * ncols; ++var )
                cout << " " << tracking_algorithms[i].second->MAP_on_var(var);
            cout << "], loc=" << tracking_algorithms[i].second->MAP_on_var(nrows * ncols) << endl;
        }

        // generate R plots
        cellmap.generate_R_plot(cout, jt, "jt");
        cellmap.generate_R_plot(cout, sir, "sir");
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

