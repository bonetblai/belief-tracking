/*
 *  Copyright (C) 2016 Universidad Simon Bolivar
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

#ifndef ORE_SLAM_PARTICLES_H
#define ORE_SLAM_PARTICLES_H

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
#include "slam_cache.h"
#include "inference.h"
#include "edbp.h"
#include "utils.h"

//#define DEBUG

namespace OreSLAM {

class cache_t : public SLAM::cache_t { };

// Abstract Particle for the Rao-Blackwellised filter
struct rbpf_particle_t : public base_particle_t {
    std::vector<int> loc_history_;

    const std::vector<int>& history() const {
        return loc_history_;
    }

    // factors
    std::vector<dai::Factor> factors_;

    // computation of marginals in factor model
    mutable std::vector<int> indices_for_updated_factors_;
    mutable std::vector<dai::Factor> marginals_;
    Inference::inference_t inference_;

    rbpf_particle_t() {
        assert(base_ != 0);
        assert(base_->nlabels_ == 2);
        factors_ = std::vector<dai::Factor>(base_->nloc_);
        marginals_ = std::vector<dai::Factor>(base_->nloc_);
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            factors_[loc] = dai::Factor(cache_t::varset(loc));
            marginals_[loc] = dai::Factor(cache_t::varset(loc));
        }

#ifdef DEBUG
        for( int loc = 0; loc < base_->nloc_; ++loc )
            inference_.print_factor(std::cout, loc, factors_, "factors");
#endif

        // create inference algorithm
        inference_.create_and_initialize_algorithm(factors_);
    }
    virtual ~rbpf_particle_t() {
        inference_.destroy_inference_algorithm();
    }

    rbpf_particle_t(const rbpf_particle_t &p) {
        *this = p;
    }

    rbpf_particle_t(rbpf_particle_t &&p)
      : loc_history_(std::move(p.loc_history_)),
        factors_(std::move(p.factors_)),
        indices_for_updated_factors_(std::move(p.indices_for_updated_factors_)),
        marginals_(std::move(p.marginals_)),
        inference_(std::move(p.inference_)) {
    }

    const rbpf_particle_t& operator=(const rbpf_particle_t &p) {
        loc_history_ = p.loc_history_;
        factors_ = p.factors_;
        indices_for_updated_factors_ = p.indices_for_updated_factors_;
        marginals_ = p.marginals_;
        inference_ = p.inference_;
        return *this;
    }

    bool operator==(const rbpf_particle_t &p) const {
        return (loc_history_ == p.loc_history_) &&
               (factors_ == p.factors_) &&
               (indices_for_updated_factors_ == p.indices_for_updated_factors_) &&
               (marginals_ == p.marginals_) &&
               (inference_ == p.inference_);
    }

    void initialize_mpi_worker(mpi_slam_t *mpi, int wid) {
#ifdef USE_MPI
        assert(mpi != 0);

        // initialize io buffers
        if( mpi->io_buffer_ == 0 )
            mpi->initialize_buffers(factors_);

        // initialize worker 
        mpi->initialize_worker(cache_t::variables(), factors_, wid);
#endif
    }

    void initial_sampling_in_place(mpi_slam_t *mpi, int wid) {
        assert(base_->nlabels_ == 2);

        // set initial history and reset factors for locations
        indices_for_updated_factors_.clear();
        indices_for_updated_factors_.reserve(base_->nloc_);
        loc_history_.push_back(base_->initial_loc_);
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / (1 << factor.vars().size());
            for( int i = 0; i < (1 << factor.vars().size()); ++i )
                factor.set(i, p);
            indices_for_updated_factors_.push_back(loc);
        }
        calculate_marginals(mpi, wid, false);
    }

    static int var_offset(int loc, int var_id) { return base_->var_offset(loc, var_id); }
    int get_slabels(int loc, const dai::VarSet &varset, int value) const {
        return cache_t::get_slabels(loc, varset, value, var_offset);
    }

    void update_factors(int last_action, int obs) {
        int current_loc = loc_history_.back();
#ifdef DEBUG
        //std::cout << "update_factors: loc=" << current_loc << ", coord=" << coord_t(current_loc) << ", obs=" << obs << std::endl;
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        inference_.print_factor(std::cout, current_loc, factors_, "factors");
#endif
        assert(current_loc < int(factors_.size()));
        dai::Factor &factor = factors_[current_loc];
        for( int value = 0; value < int(factor.nrStates()); ++value ) {
            int slabels = get_slabels(current_loc, factor.vars(), value);
            factor.set(value, factor[value] * base_->probability_obs(obs, current_loc, slabels, last_action));
        }
        factor.normalize();
        indices_for_updated_factors_.push_back(current_loc);
#ifdef DEBUG
        std::cout << "factor after  update: loc=" << current_loc << ", obs=" << obs << std::endl;
        inference_.print_factor(std::cout, current_loc, factors_, "factors");
#endif
    }

    void calculate_marginals(mpi_slam_t *mpi, int wid, bool print_marginals = false) const {
#ifndef USE_MPI
        inference_.calculate_marginals(cache_t::variables(),
                                       indices_for_updated_factors_,
                                       factors_,
                                       marginals_,
                                       Inference::edbp_t::edbp_factor_index,
                                       print_marginals);
#else
        assert(mpi != 0);
        assert((wid > 0) && (wid < mpi->nworkers_));
        mpi->calculate_marginals(factors_, indices_for_updated_factors_, wid);
        assert(indices_for_updated_factors_.empty());
#endif
    }

    void update_marginals(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            dai::Factor marginal = marginals_[loc].marginal(dai::Var(loc, 2));
            assert(base_->nlabels_ == int(marginal.nrStates()));
            for( int label = 0; label < base_->nlabels_; ++label )
                marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * marginal[label]);
        }
        int current_loc = loc_history_.back();
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
    }

    int value_for(int /*var*/) const { return -1; }

    void mpi_update_marginals(mpi_slam_t *mpi, int wid) {
#ifdef USE_MPI
        mpi->read_marginals_from_worker(marginals_, wid);
#endif
    }

    void print(std::ostream &os) const {
        if( loc_history_.empty() )
            os << "loc=<empty history>";
        else
            os << "loc=" << loc_history_.back();
    }
};

// Particle for the motion model RBPF filter
struct motion_model_rbpf_particle_t : public rbpf_particle_t {
#if 0 // for some reason, it runs faster without these...
    motion_model_rbpf_particle_t() : rbpf_particle_t() { }
    ~motion_model_rbpf_particle_t() { }

    motion_model_rbpf_particle_t(const motion_model_rbpf_particle_t &p) {
        *this = p;
    }

    motion_model_rbpf_particle_t(motion_model_rbpf_particle_t &&p) : rbpf_particle_t(std::move(p)) {
    }

    const motion_model_rbpf_particle_t& operator=(const motion_model_rbpf_particle_t &p) {
        *static_cast<rbpf_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const motion_model_rbpf_particle_t &p) const {
        return *static_cast<const rbpf_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("mm_rbpf2_sir");
    }

    virtual bool sample_from_pi(rbpf_particle_t &np,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                mpi_slam_t *mpi,
                                int wid) const {
#ifdef DEBUG
        assert(*this == np);
#endif
        int next_loc = base_->sample_loc(loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        if( !history_container.contains(np.loc_history_) ) {
            // this is a new loc history, perform update
            np.update_factors(last_action, obs);
            np.calculate_marginals(mpi, wid, false);
        }
        return true; // CHECK: what happens with incompatible obs?
    }

    virtual float importance_weight(const rbpf_particle_t &np, int last_action, int obs) const {
        assert(indices_for_updated_factors_.empty());
        int np_current_loc = np.loc_history_.back();
        float weight = 0;
        const dai::Factor &marginal = marginals_[np_current_loc];
        for( int value = 0; value < int(marginal.nrStates()); ++value ) {
            int slabels = get_slabels(np_current_loc, marginal.vars(), value);
            weight += marginal[value] * base_->probability_obs(obs, np_current_loc, slabels, last_action);
        }
        return weight;
    }

    motion_model_rbpf_particle_t* initial_sampling(mpi_slam_t *mpi, int wid) {
        motion_model_rbpf_particle_t *p = new motion_model_rbpf_particle_t;
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

// Particle for the optimal RBPF filter (verified: 09/12/2015)
struct optimal_rbpf_particle_t : public rbpf_particle_t {
    mutable std::vector<float> cdf_;

#if 0 // for some reason, it runs faster without these...
    optimal_rbpf_particle_t() : rbpf_particle_t() { }
    ~optimal_rbpf_particle_t() { }

    optimal_rbpf_particle_t(const optimal_rbpf_particle_t &p) {
        *this = p;
    }

    optimal_rbpf_particle_t(optimal_rbpf_particle_t &&p) : rbpf_particle_t(std::move(p)) {
    }

    const optimal_rbpf_particle_t& operator=(const optimal_rbpf_particle_t &p) {
        *static_cast<rbpf_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const optimal_rbpf_particle_t &p) const {
        return *static_cast<const rbpf_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("opt_rbpf2_sir");
    }

    void calculate_cdf(int last_action, int obs, std::vector<float> &cdf) const {
        // make sure there is no pending inference on factor model
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
                prob += p_marginal[value] * base_->probability_obs(obs, nloc, slabels, last_action);
            }
            cdf.push_back(previous + base_->probability_tr_loc(last_action, current_loc, nloc) * prob);
            previous = cdf.back();
        }

        // normalize (i.e. calculate alpha)
        assert(cdf.back() > 0);
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            cdf[nloc] /= cdf.back();
        }
    }

    virtual bool sample_from_pi(rbpf_particle_t &np,
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
            np.calculate_marginals(mpi, wid, false);
        }
        return true; // CHECK: what happens with incompatible obs?
    }

    virtual float importance_weight(const rbpf_particle_t &, int last_action, int obs) const {
        float weight = 0;
        int current_loc = loc_history_.back();
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            float p = 0;
            const dai::Factor &marginal = marginals_[current_loc];
            for( int value = 0; value < int(marginal.nrStates()); ++value ) {
                int slabels = get_slabels(current_loc, marginal.vars(), value);
                p += marginal[value] * base_->probability_obs(obs, nloc, slabels, last_action);
            }
            weight += base_->probability_tr_loc(last_action, current_loc, nloc) * p;
        }
        return weight;
    }

    optimal_rbpf_particle_t* initial_sampling(mpi_slam_t *mpi, int wid) {
        optimal_rbpf_particle_t *p = new optimal_rbpf_particle_t;
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

}; // namespace OreSLAM

#undef DEBUG

#endif

