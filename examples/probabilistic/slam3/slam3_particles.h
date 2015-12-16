/*
 *  Copyright (C) 2015 Universidad Simon Bolivar
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

#ifndef SLAM3_PARTICLES_H
#define SLAM3_PARTICLES_H

#include <cassert>
#include <cstdlib>
#include <string>
#include <iostream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>

#include <dai/alldai.h>

#include "cellmap.h"
#include "slam_particles.h"
#include "inference.h"
#include "utils.h"

#include "var_beam.h"
#include "arc_consistency.h"

//#define DEBUG

class varset_beam_t : public var_beam_t {
  protected:
    const int loc_;
    const dai::Var var_;
    const dai::VarSet varset_;
    unsigned mask_;

    float alpha_;
    static float kappa_;

  public:
    varset_beam_t(int loc, const dai::Var &var, const dai::VarSet &varset)
      : var_beam_t(1, varset.nrStates()), loc_(loc), var_(var), varset_(varset), mask_(0), alpha_(-1) {
        mask_ = unsigned(-1);
        for( dai::State state(varset_); state.valid(); state++ ) {
            if( state(var_) == 1 )
                mask_ = mask_ & dai::calcLinearState(varset_, state);
        }
#ifdef DEBUG
        std::cout << "varset_beam_t: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    varset_beam_t(const varset_beam_t &beam)
      : var_beam_t(beam),
        loc_(beam.loc_),
        var_(beam.var_),
        varset_(beam.varset_),
        mask_(beam.mask_),
        alpha_(beam.alpha_) {
#ifdef DEBUG
        std::cout << "varset_beam_t: copy-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    varset_beam_t(varset_beam_t &&beam)
      : var_beam_t(std::move(beam)),
        loc_(beam.loc_),
        var_(beam.var_),
        varset_(std::move(beam.varset_)),
        mask_(beam.mask_),
        alpha_(beam.alpha_) {
#ifdef DEBUG
        std::cout << "varset_beam_t: move-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    ~varset_beam_t() { }

    static float kappa() { return kappa_; }
    static void set_kappa(float kappa) { kappa_ = kappa; }

    bool operator==(const varset_beam_t &beam) const {
        return (loc_ == beam.loc_) && (*static_cast<const var_beam_t*>(this) == *static_cast<const var_beam_t*>(&beam));
    }

    int loc() const { return loc_; }
    const dai::VarSet& varset() const { return varset_; }

    float probability(int valuation) const {
        // need to consider case where valuation is inside/outside csp. In the former case,
        // probability is alpha / number-valuations-inside. In the latter case, it is 
        // alpha * kappa / number-valuations-outside.
        float p = contains(valuation) ? 1.0 : kappa_;
        //std::cout << "probability: p=" << p << ", mask=" << mask_ << ", alpha=" << alpha_ << std::endl;
        return alpha_ * p;
    }
    float marginal(int label) const {
        float p = 0;
        for( unsigned valuation = 0; valuation < unsigned(varset_.nrStates()); ++valuation ) {
            // if valuation is compatible with lab, p += probability(valuation)
            if( ((label == 1) && ((valuation & mask_) != 0)) || ((label == 0) && ((valuation & mask_) == 0)) )
                p += probability(valuation);
        }
        return p;
    }

    int label(const dai::Var &var, int valuation) const {
        std::map<dai::Var, size_t> state;
        state = dai::calcState(varset_, valuation);
        return state[var];
    }

    void set_alpha() {
        alpha_ = 1.0 / float(size() + (varset_.nrStates() - size()) * kappa_);
        //std::cout << "set-alpha: #I=" << size() << ", #E=" << varset_.nrStates() - size() << ", kappa=" << kappa_ << ", alpha=" << alpha_ << std::endl;
    }
    void set_initial_configuration() {
        var_beam_t::set_initial_configuration();
        set_alpha();
    }
    void erase_ordered_indices(const std::vector<int> &ordered_indices) {
        var_beam_t::erase_ordered_indices(ordered_indices);
    }
};

class arc_consistency_t : public CSP::arc_consistency_t<varset_beam_t> {
    static CSP::constraint_digraph_t cg_;

    mutable std::map<dai::Var, size_t> state_x_;

    virtual void arc_reduce_preprocessing_0(int var_x, int var_y) { }
    virtual void arc_reduce_preprocessing_1(int var_x, int val_x) {
        state_x_ = dai::calcState(domain_[var_x]->varset(), val_x);
    }
    virtual void arc_reduce_postprocessing(int var_x, int var_y) { }
    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const {
        std::map<dai::Var, size_t> state_y = dai::calcState(domain_[var_y]->varset(), val_y);
        for( std::map<dai::Var, size_t>::const_iterator it = state_x_.begin(); it != state_x_.end(); ++it ) {
            std::map<dai::Var, size_t>::const_iterator jt = state_y.find(it->first);
            if( (jt != state_y.end()) && (it->second != jt->second) ) return false;
        }
        return true;
    }

  public:
    arc_consistency_t() : CSP::arc_consistency_t<varset_beam_t>(cg_) { }
    arc_consistency_t(const arc_consistency_t &ac) = delete;
    virtual ~arc_consistency_t() { }

    static void initialize_constraint_graph(int nrows, int ncols) {
        construct_constraint_graph(nrows, ncols);
    }

    static void construct_constraint_graph(int nrows, int ncols) {
        cg_.create_empty_graph(nrows * ncols);
        for( int loc = 0; loc < nrows * ncols; ++loc ) {
            cg_.reserve_edge_list(loc, 8);
            int r = loc / ncols, c = loc % ncols;
            for( int dr = -1; dr < 2; ++dr ) {
                if( (r + dr < 0) || (r + dr >= nrows) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    if( (c + dc < 0) || (c + dc >= ncols) ) continue;
                    if( (dr == 0) && (dc == 0) ) continue;
                    int nloc = (r + dr) * ncols + (c + dc);
                    cg_.add_edge(loc, nloc);
                }
            }
        }
        std::cout << "# cg: #vars=" << cg_.nvars() << ", #edges=" << cg_.nedges() << std::endl;
    }

    const arc_consistency_t& operator=(const arc_consistency_t &ac) {
        nvars_ = ac.nvars_;
        assert(domain_.size() == ac.domain_.size());
        for( int loc = 0; loc < int(ac.domain_.size()); ++loc )
            set_domain(loc, new varset_beam_t(*ac.domain(loc)));
        return *this;
    }
    bool operator==(const arc_consistency_t &ac) const {
        if( (nvars_ == ac.nvars_) && (domain_.size() == ac.domain_.size()) ) {
            for( int loc = 0; loc < int(domain_.size()); ++loc ) {
                if( !(*domain_[loc] == *ac.domain_[loc]) )
                    return false;
            }
            return true;
        } else {
            return false;
        }
    }

    void ac3(std::vector<int> &revised_vars, bool propagate = true) {
        CSP::arc_consistency_t<varset_beam_t>::ac3(revised_vars, propagate);
        set_alpha();
    }

    void set_alpha() {
        for( int loc = 0; loc < nvars_; ++loc )
            domain_[loc]->set_alpha();
    }

    void print_factor(std::ostream &os, int loc) const { }
    void print(std::ostream &os) const { }
};

// Abstract Particle for the 2nd Rao-Blackwellised filter
struct rbpf_slam3_particle_t : public base_particle_t {
    std::vector<int> loc_history_;

    const std::vector<int>& history() const {
        return loc_history_;
    }

    // arc consistency
    arc_consistency_t csp_;
    mutable std::vector<int> revised_variables_;

    // cache for conversion  from factor values into slabels (it is dynamically filled by get_slabels())
    static std::vector<std::vector<int> > slabels_;

    rbpf_slam3_particle_t() {
        assert(base_ != 0);
        assert(base_->nlabels_ == 2);

        // create binary variables for each cell in the grid
        int nloc = base_->nloc_;
        std::vector<dai::Var> variables(nloc);
        for( int loc = 0; loc < nloc; ++loc )
            variables[loc] = dai::Var(loc, 2);

        // create one factor for each location. The variables
        // in the factor are the variables for the location
        // surrounding the factor, including the variable
        // for the "center" location. Also set up the center
        // for each factor.
        for( int loc = 0; loc < nloc; ++loc ) {
            int row = loc / base_->ncols_, col = loc % base_->ncols_;
            std::vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= base_->nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= base_->ncols_) ) continue;
                    vars.push_back(variables[nr * base_->ncols_ + nc]);
                }
            }
            std::sort(vars.begin(), vars.end());
            dai::VarSet varset(vars.begin(), vars.end());
            csp_.set_domain(loc, new varset_beam_t(loc, variables[loc], varset));
        }
    }
    rbpf_slam3_particle_t(const rbpf_slam3_particle_t &p)
      : loc_history_(p.loc_history_) {
        csp_ = p.csp_;
    }
#if 0
    rbpf_slam3_particle_t(rbpf_slam3_particle_t &&p)
      : loc_history_(std::move(p.loc_history_)), csp_(std::move(p.csp_)) {
    }
#endif
    virtual ~rbpf_slam3_particle_t() {
        csp_.delete_domains_and_clear();
    }

    const rbpf_slam3_particle_t& operator=(const rbpf_slam3_particle_t &p) {
        loc_history_ = p.loc_history_;
        csp_ = p.csp_;
        return *this;
    }

    bool operator==(const rbpf_slam3_particle_t &p) const {
        return (loc_history_ == p.loc_history_) && (csp_ == p.csp_);
    }

    void reset_csp() {
        for( int loc = 0; loc < base_->nloc_; ++loc )
            csp_.domain(loc)->set_initial_configuration();
    }

    void initial_sampling_in_place(mpi_slam_t *mpi, int wid) {
        assert(base_->nlabels_ == 2);
        loc_history_.push_back(base_->initial_loc_);
        reset_csp();
        assert(csp_.is_consistent(0));
    }

    int get_slabels(int loc, const dai::VarSet &vars, int value) const {
        assert((loc >= 0) && (loc < base_->nloc_));
        assert((value >= 0) && (value < int(vars.nrStates())));

        // allocate cache if this is first call
        if( slabels_.empty() )
            slabels_ = std::vector<std::vector<int> >(base_->nloc_, std::vector<int>(512, -1));
        assert(loc < int(slabels_.size()));
        if( slabels_[loc].empty() )
            slabels_[loc] = std::vector<int>(vars.nrStates(), -1);
        assert(value < int(slabels_[loc].size()));

        // check whether there is a valid entry in cache
        const std::vector<int> &slabels_for_loc = slabels_[loc];
        if( slabels_for_loc[value] != -1 )
            return slabels_for_loc[value];

#ifdef DEBUG
        std::cout << "get_slabels(loc=" << loc << ":" << coord_t(loc) << ", value=" << value << "):" << std::endl;
#endif

        // this is the first time that we access (loc,value)
        // compute the correct value and cache it for later use
        int slabels = 0;
        std::map<dai::Var, size_t> states = dai::calcState(vars, value);
        for( dai::VarSet::const_iterator it = vars.begin(); it != vars.end(); ++it ) {
            const dai::Var &var = *it;
            size_t var_value = states[var];
            assert(var_value < 2); // because it is a binary variable
            if( var_value ) {
                int var_id = it->label();
                int var_off = base_->var_offset(loc, var_id);
                assert((var_off >= 0) && (var_off < 9));
#ifdef DEBUG
                std::cout << "    [var_id=" << var_id << ":" << coord_t(var_id)
                          << ", var_value=1, off=" << var_off << "]"
                          << std::endl;
#endif
                slabels += (1 << var_off);
            }
        }
#ifdef DEBUG
        std::cout << "    bits=|";
        print_bits(std::cout, slabels, 9);
        std::cout << "| (" << slabels << ")" << std::endl;
#endif

        // cache it and return
        slabels_[loc][value] = slabels;
        return slabels;
    }

    void update_factors(int last_action, int obs) {
        int current_loc = loc_history_.back();
#ifdef DEBUG
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        csp_.print_factor(std::cout, current_loc);
#endif
        assert(current_loc < int(csp_.nvars()));
        std::vector<int> indices_to_be_removed;
        varset_beam_t &beam = *csp_.domain(current_loc);
        const dai::VarSet &varset = beam.varset();
        for( varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            int value = *it;
            int index = it.index();
            int slabels = get_slabels(current_loc, varset, value);
            float p = base_->probability_obs_ore_slam(obs, current_loc, slabels, last_action);
            if( p < varset_beam_t::kappa() ) indices_to_be_removed.push_back(index);
        }

        if( !indices_to_be_removed.empty() ) {
            beam.erase_ordered_indices(indices_to_be_removed);
            csp_.add_to_worklist(current_loc);
            std::cout << "[ac3..." << std::flush;
            csp_.ac3(revised_variables_);
            std::cout << "done]" << std::flush;
            assert(csp_.is_consistent(0));
        }
#ifdef DEBUG
        std::cout << "factor after  update: loc=" << current_loc << ", obs=" << obs << std::endl;
        csp_.print_factor(std::cout, current_loc);
#endif
    }

    void update_marginals(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            const varset_beam_t &beam = *csp_.domain(loc);
            for( int label = 0; label < base_->nlabels_; ++label )
                marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * beam.marginal(label));
        }
        int current_loc = loc_history_.back();
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
    }

    int value_for(int /*var*/) const { return -1; }

    void print(std::ostream &os) const {
        if( loc_history_.empty() )
            os << "loc=<empty history>";
        else
            os << "loc=" << loc_history_.back();
    }
};

// Particle for the motion model RBPF filter (slam3)
struct motion_model_rbpf_slam3_particle_t : public rbpf_slam3_particle_t {

    motion_model_rbpf_slam3_particle_t() : rbpf_slam3_particle_t() { }
    motion_model_rbpf_slam3_particle_t(const motion_model_rbpf_slam3_particle_t &p)
      : rbpf_slam3_particle_t(p) {
    }
    motion_model_rbpf_slam3_particle_t(motion_model_rbpf_slam3_particle_t &&p)
      : rbpf_slam3_particle_t(std::move(p)) {
    }
    ~motion_model_rbpf_slam3_particle_t() { }

    const motion_model_rbpf_slam3_particle_t& operator=(const motion_model_rbpf_slam3_particle_t &p) {
        *static_cast<rbpf_slam3_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const motion_model_rbpf_slam3_particle_t &p) const {
        return *static_cast<const rbpf_slam3_particle_t*>(this) == p;
    }

    static std::string type() {
        return std::string("mm_rbpf3_sir");
    }

    virtual bool sample_from_pi(rbpf_slam3_particle_t &np,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                mpi_slam_t *mpi,
                                int wid) const {
#ifdef DEBUG
        assert(np == *this);
#endif
        int next_loc = base_->sample_loc(loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        if( !history_container.contains(np.loc_history_) ) {
            // this is a new loc history, perform update
            np.update_factors(last_action, obs);
        }
        return true; // CHECK: what happens with incompatible obs?
    }

    virtual float importance_weight(const rbpf_slam3_particle_t &np, int last_action, int obs) const {
        int np_current_loc = np.loc_history_.back();
        float weight = 0;
        const varset_beam_t &beam = *csp_.domain(np_current_loc);
        const dai::VarSet &varset = beam.varset();
        for( int value = 0; value < int(varset.nrStates()); ++value ) {
            int slabels = get_slabels(np_current_loc, varset, value);
            weight += beam.probability(value) * base_->probability_obs_ore_slam(obs, np_current_loc, slabels, last_action);
        }
        return weight;
    }

    motion_model_rbpf_slam3_particle_t* initial_sampling(mpi_slam_t *mpi, int wid) {
        motion_model_rbpf_slam3_particle_t *p = new motion_model_rbpf_slam3_particle_t;
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

// Particle for the optimal RBPF filter (verified: 09/12/2015)
struct optimal_rbpf_slam3_particle_t : public rbpf_slam3_particle_t {
    mutable std::vector<float> cdf_;

#if 0 // for some reason, it runs faster without these...
    optimal_rbpf_slam3_particle_t() : rbpf_slam3_particle_t() { }
    ~optimal_rbpf_slam3_particle_t() { }

    optimal_rbpf_slam3_particle_t(const optimal_rbpf_slam3_particle_t &p) {
        *this = p;
    }

    optimal_rbpf_slam3_particle_t(optimal_rbpf_slam3_particle_t &&p) : rbpf_slam3_particle_t(std::move(p)) {
    }

    const optimal_rbpf_slam3_particle_t& operator=(const optimal_rbpf_slam3_particle_t &p) {
        *static_cast<rbpf_slam3_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const optimal_rbpf_slam3_particle_t &p) const {
        return *static_cast<const rbpf_slam3_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("opt_rbpf3_sir");
    }

    void calculate_cdf(int last_action, int obs, std::vector<float> &cdf) const {
        // make sure there is no pending inference on factor model
 #if 0
        assert(indices_for_updated_factors_.empty());

        cdf.clear();
        cdf.reserve(base_->nloc_);

        // P(nloc | loc, action, obs) = alpha * P(nloc, obs | loc, action)
        //                            = alpha * P(obs | nloc, loc, action) * P(nloc | loc, action)
        //                            = alpha * P(obs | nloc, action) * P(nloc | loc, action)

        float previous = 0;
        int current_loc = loc_history_.back();
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            const dai::Factor &p_marginal = marginals_[nloc];
            float prob = 0;
            for( int value = 0; value < int(p_marginal.nrStates()); ++value ) {
                int slabels = get_slabels(nloc, p_marginal.vars(), value);
                prob += p_marginal[value] * base_->probability_obs_ore_slam(obs, nloc, slabels, last_action);
            }
            cdf.push_back(previous + base_->probability_tr_loc(last_action, current_loc, nloc) * prob);
            previous = cdf.back();
        }

        // normalize (i.e. calculate alpha)
        assert(cdf.back() > 0);
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            cdf[nloc] /= cdf.back();
        }
#endif
        assert(0);
    }

    virtual bool sample_from_pi(rbpf_slam3_particle_t &np,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                mpi_slam_t *mpi,
                                int wid) const {
#ifdef DEBUG
        assert(*this == np);
#endif
        calculate_cdf(last_action, obs, cdf_);
        int next_loc = Utils::sample_from_distribution(base_->nloc_, &cdf_[0]);
        np.loc_history_.push_back(next_loc);
        if( !history_container.contains(np.loc_history_) ) {
            // this is a new loc history, perform update
            np.update_factors(last_action, obs);
        }
        return true; // CHECK: what happens with incompatible obs?
    }

    virtual float importance_weight(const rbpf_slam3_particle_t &, int last_action, int obs) const {
        float weight = 0;
        int current_loc = loc_history_.back();
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            const varset_beam_t &beam = *csp_.domain(current_loc);
            const dai::VarSet &varset = beam.varset();
            float p = 0;
            for( int value = 0; value < int(varset.nrStates()); ++value ) {
                int slabels = get_slabels(current_loc, varset, value);
                p += beam.probability(value) * base_->probability_obs_ore_slam(obs, nloc, slabels, last_action);
            }
            weight += base_->probability_tr_loc(last_action, current_loc, nloc) * p;
        }
        return weight;
    }

    optimal_rbpf_slam3_particle_t* initial_sampling(mpi_slam_t *mpi, int wid) {
        optimal_rbpf_slam3_particle_t *p = new optimal_rbpf_slam3_particle_t;
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

#undef DEBUG

#endif

