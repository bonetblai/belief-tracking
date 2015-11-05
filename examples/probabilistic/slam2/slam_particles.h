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

#ifndef SLAM_PARTICLES_H
#define SLAM_PARTICLES_H

#include <cassert>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <stdlib.h>

#include <dai/alldai.h>

#include "cellmap.h"
#include "utils.h"

#include "mpi_slam.h"

#define DEBUG

// Generic particle for the color-tile SLAM problem
struct base_particle_t {
    static const cellmap_t *base_;
    const std::vector<int>& history() const {
        assert(0);
    }
};

struct slam_particle_t : public base_particle_t {
    int current_loc_;
    std::vector<int> labels_;

    slam_particle_t() { }
    ~slam_particle_t() { }

    slam_particle_t(const slam_particle_t &p) {
        *this = p;
    }

    slam_particle_t(slam_particle_t &&p)
      : current_loc_(p.current_loc_), labels_(std::move(p.labels_)) {
    }

    const slam_particle_t& operator=(const slam_particle_t &p) {
        current_loc_ = p.current_loc_;
        labels_ = p.labels_;
        return *this;
    }

    bool operator==(const slam_particle_t &p) const {
        return (current_loc_ == p.current_loc_) && (labels_ == p.labels_);
    }

    void initial_sampling_in_place() {
        labels_ = std::vector<int>(base_->nloc_);
        for( int i = 0; i < base_->nloc_; ++i )
            labels_[i] = lrand48() % base_->nlabels_;
        current_loc_ = base_->initial_loc_;
    }

    int value_for(int var) const {
        return var == base_->nvars_ - 1 ? current_loc_ : labels_[var];
    }

    void initialize_mpi_worker(mpi_slam_t * /*mpi*/, int /*wid*/) { } 
    void mpi_update_marginals(mpi_slam_t * /*mpi*/, int /*wid*/) { }
};

// Particle for the SIS filter
struct sis_slam_particle_t : public slam_particle_t {
#if 0 // for some reason, it runs faster without these...
    sis_slam_particle_t() : slam_particle_t() { }
    ~sis_slam_particle_t() { }

    sis_slam_particle_t(const sis_slam_particle_t &p) {
        *this = p;
    }

    sis_slam_particle_t(sis_slam_particle_t &&p) : slam_particle_t(std::move(p)) {
    }

    const sis_slam_particle_t& operator=(const sis_slam_particle_t &p) {
        *static_cast<slam_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const sis_slam_particle_t &p) const {
        return *static_cast<const slam_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("sis");
    }

    void update(float &weight, int last_action, int obs) {
        int new_loc = base_->sample_loc(current_loc_, last_action);
        weight *= base_->probability_obs_standard(obs, new_loc, labels_[new_loc], last_action);
        current_loc_ = new_loc;
    }

    sis_slam_particle_t* initial_sampling() {
        sis_slam_particle_t *p = new sis_slam_particle_t;
        p->initial_sampling_in_place();
        return p;
    }
};

// Particle for the motion model SIR filter
struct motion_model_sir_slam_particle_t : public slam_particle_t {
#if 0 // for some reason, it runs faster without these...
    motion_model_sir_slam_particle_t() : slam_particle_t() { }
    ~motion_model_sir_slam_particle_t() { }

    motion_model_sir_slam_particle_t(const motion_model_sir_slam_particle_t &p) {
        *this = p;
    }

    motion_model_sir_slam_particle_t(motion_model_sir_slam_particle_t &&p) : slam_particle_t(std::move(p)) {
    }

    const motion_model_sir_slam_particle_t& operator=(const motion_model_sir_slam_particle_t &p) {
        *static_cast<slam_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const motion_model_sir_slam_particle_t &p) const {
        return *static_cast<const slam_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("mm_sir");
    }

    void sample_from_pi(motion_model_sir_slam_particle_t &np,
                        const motion_model_sir_slam_particle_t &p,
                        int last_action,
                        int /*obs*/) const {
#ifdef DEBUG
        assert(np == p);
#endif
        np.current_loc_ = base_->sample_loc(p.current_loc_, last_action);
    }

    float importance_weight(const motion_model_sir_slam_particle_t &np,
                            const motion_model_sir_slam_particle_t &/*p*/,
                            int last_action,
                            int obs) const {
        return base_->probability_obs_standard(obs, np.current_loc_, np.labels_, last_action);
    }

    motion_model_sir_slam_particle_t* initial_sampling(mpi_slam_t * /*mpi*/, int /*wid*/) {
        motion_model_sir_slam_particle_t *p = new motion_model_sir_slam_particle_t;
        p->initial_sampling_in_place();
        return p;
    }
};

// Particle for the optimal SIR filter
struct optimal_sir_slam_particle_t : public slam_particle_t {
#if 0 // for some reason, it runs faster without these...
    optimal_sir_slam_particle_t() : slam_particle_t() { }
    ~optimal_sir_slam_particle_t() { }

    optimal_sir_slam_particle_t(const optimal_sir_slam_particle_t &p) {
        *this = p;
    }

    optimal_sir_slam_particle_t(optimal_sir_slam_particle_t &&p) : slam_particle_t(std::move(p)) {
    }

    const optimal_sir_slam_particle_t& operator=(const optimal_sir_slam_particle_t &p) {
        *static_cast<slam_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const optimal_sir_slam_particle_t &p) const {
        return *static_cast<const slam_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("opt_sir");
    }

    // encode/decode particles into/from integers
    int encode() const {
        int code = 0;
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            code *= base_->nlabels_;
            code += labels_[loc];
        }
        code = code * base_->nloc_ + current_loc_;
        return code;
    }
    void decode(int code) {
        labels_ = std::vector<int>(base_->nloc_, 0);
        current_loc_ = code % base_->nloc_;
        code /= base_->nloc_;
        for( int loc = base_->nloc_ - 1; loc >= 0; --loc ) {
            labels_[loc] = code % base_->nlabels_;
            code /= base_->nlabels_;
        }
    }

    float pi(const optimal_sir_slam_particle_t &np,
             const optimal_sir_slam_particle_t &p,
             int last_action,
             int obs) const {
        // pi(np|p,last_action,obs) = P(np|p,last_action,obs)
        //                          = P(np,obs|p,last_action) / P(obs|p,last_action)
        //                          = P(obs|np,p,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        //                          = P(obs|np,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        if( np.labels_ == p.labels_ ) {
            float prob = base_->probability_obs_standard(obs, np.current_loc_, np.labels_, last_action);
            prob *= base_->probability_tr_loc(last_action, p.current_loc_, np.current_loc_);
            return prob / importance_weight(np, p, last_action, obs);
        } else {
            return 0;
        }
    }

    float importance_weight(const optimal_sir_slam_particle_t &/*np*/,
                            const optimal_sir_slam_particle_t &p,
                            int last_action,
                            int obs) const {
        // weight = P(obs|np,last_action) * P(np|p,last_action) / pi(np|p,last_action,obs)
        //        = P(obs|np,last_action) * P(np|p,last_action) / P(np|p,last_action,obs)
        //        = P(obs|p,last_action) [see above derivation in pi(..)]
        //        = SUM_{np} P(np,obs|p,last_action)
        //        = SUM_{np} P(obs|np,p,last_action) * P(np|p,last_action)
        //        = SUM_{np} P(obs|np,last_action) * P(np|p,last_action)
        // since P(np|p,last_action) = 0 if p and np have different labels,
        // we can simplify the sum over locations instead of maps
        float weight = 0;
        int loc = p.current_loc_;
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc )
            weight += base_->probability_obs_standard(obs, new_loc, p.labels_, last_action) * base_->probability_tr_loc(last_action, loc, new_loc);
        return weight;
    }

    optimal_sir_slam_particle_t* initial_sampling(mpi_slam_t * /*mpi*/, int /*wid*/) {
        optimal_sir_slam_particle_t *p = new optimal_sir_slam_particle_t;
        p->initial_sampling_in_place();
        return p;
    }
};

// Abstract Particle for the Rao-Blackwellised filter
struct rbpf_slam_particle_t : public base_particle_t {
    std::vector<int> loc_history_;
    std::vector<dai::Factor> factors_;

    const std::vector<int>& history() const {
        return loc_history_;
    }

    rbpf_slam_particle_t() { }
    virtual ~rbpf_slam_particle_t() { }

    rbpf_slam_particle_t(const rbpf_slam_particle_t &p) {
        *this = p;
    }

    rbpf_slam_particle_t(rbpf_slam_particle_t &&p)
      : loc_history_(std::move(p.loc_history_)), factors_(std::move(p.factors_)) {
    }

    const rbpf_slam_particle_t& operator=(const rbpf_slam_particle_t &p) {
        loc_history_ = p.loc_history_;
        factors_ = p.factors_;
        return *this;
    }

    bool operator==(const rbpf_slam_particle_t &p) const {
        return (loc_history_ == p.loc_history_) && (factors_ == p.factors_);
    }

    void initial_sampling_in_place() {
        loc_history_.push_back(base_->initial_loc_);
        factors_ = std::vector<dai::Factor>(base_->nloc_, dai::Factor(dai::VarSet(dai::Var(0, base_->nlabels_)), 1.0 / float(base_->nlabels_)));
    }

    void update_factors(int last_action, int obs) {
        int current_loc = loc_history_.back();
        assert(current_loc < int(factors_.size()));
        dai::Factor &factor = factors_[current_loc];
        assert(base_->nlabels_ == int(factor.nrStates()));
        for( int label = 0; label < base_->nlabels_; ++label )
            factor.set(label, factor[label] * base_->probability_obs_standard(obs, current_loc, label, last_action));
        factor.normalize();
    }

    float probability(int label, int loc) const {
        assert(loc < int(factors_.size()));
        assert(label < int(factors_[loc].nrStates()));
        return factors_[loc][label];
    }

    void update_marginals(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        int current_loc = loc_history_.back();
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            for( int label = 0; label < base_->nlabels_; ++label )
                marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * probability(label, loc));
        }
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
    }

    int value_for(int /*var*/) const { return -1; }

    virtual void sample_from_pi(rbpf_slam_particle_t &np,
                                const rbpf_slam_particle_t &p,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                mpi_slam_t *mpi,
                                int wid) const = 0;
    virtual float importance_weight(const rbpf_slam_particle_t &np,
                                    const rbpf_slam_particle_t &p,
                                    int last_action,
                                    int obs) const = 0;

    void initialize_mpi_worker(mpi_slam_t * /*mpi*/, int /*wid*/) { } 
    void mpi_update_marginals(mpi_slam_t * /*mpi*/, int /*wid*/) { }
};

// Particle for the motion model RBPF filter
struct motion_model_rbpf_slam_particle_t : public rbpf_slam_particle_t {
#if 0 // for some reason, it runs faster without these...
    motion_model_rbpf_slam_particle_t() : rbpf_slam_particle_t() { }
    ~motion_model_rbpf_slam_particle_t() { }

    motion_model_rbpf_slam_particle_t(const motion_model_rbpf_slam_particle_t &p) {
        *this = p;
    }

    motion_model_rbpf_slam_particle_t(motion_model_rbpf_slam_particle_t &&p) : rbpf_slam_particle_t(std::move(p)) {
    }

    const motion_model_rbpf_slam_particle_t& operator=(const motion_model_rbpf_slam_particle_t &p) {
        *static_cast<rbpf_slam_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const motion_model_rbpf_slam_particle_t &p) const {
        return *static_cast<const rbpf_slam_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("mm_rbpf_sir");
    }

    virtual void sample_from_pi(rbpf_slam_particle_t &np,
                                const rbpf_slam_particle_t &p,
                                int last_action,
                                int obs,
                                const history_container_t &/*history_container*/,
                                mpi_slam_t * /*mpi*/,
                                int /*wid*/) const {
#ifdef DEBUG
        assert(np == p);
#endif
        int next_loc = base_->sample_loc(p.loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        np.update_factors(last_action, obs);
    }

    virtual float importance_weight(const rbpf_slam_particle_t &np,
                                    const rbpf_slam_particle_t &p,
                                    int last_action,
                                    int obs) const {
        int np_current_loc = np.loc_history_.back();
        float weight = 0;
        for( int label = 0; label < base_->nlabels_; ++label ) // marginalize over possible labels at current loc
            weight += base_->probability_obs_standard(obs, np_current_loc, label, last_action) * p.probability(label, np_current_loc);
        return weight;
    }

    motion_model_rbpf_slam_particle_t* initial_sampling(mpi_slam_t * /*mpi*/, int /*wid*/) {
        motion_model_rbpf_slam_particle_t *p = new motion_model_rbpf_slam_particle_t;
        p->initial_sampling_in_place();
        return p;
    }
};

// Particle for the optimal RBPF filter
struct optimal_rbpf_slam_particle_t : public rbpf_slam_particle_t {
    mutable std::vector<float> cdf_;

#if 0 // for some reason, it runs faster without these...
    optimal_rbpf_slam_particle_t() : rbpf_slam_particle_t() { }
    ~optimal_rbpf_slam_particle_t() { }

    optimal_rbpf_slam_particle_t(const optimal_rbpf_slam_particle_t &p) {
        *this = p;
    }

    optimal_rbpf_slam_particle_t(optimal_rbpf_slam_particle_t &&p) : rbpf_slam_particle_t(std::move(p)) {
    }

    const optimal_rbpf_slam_particle_t& operator=(const optimal_rbpf_slam_particle_t &p) {
        *static_cast<rbpf_slam_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const optimal_rbpf_slam_particle_t &p) const {
        return *static_cast<const rbpf_slam_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("opt_rbpf_sir");
    }

    void calculate_cdf(const rbpf_slam_particle_t &p, int last_action, int obs, std::vector<float> &cdf) const {
        cdf.clear();
        cdf.reserve(base_->nloc_);

        // P(nloc | loc, action, obs) = alpha * P(nloc, obs | loc, action)
        //                            = alpha * P(obs | nloc, loc, action) * P(nloc | loc, action)
        //                            = alpha * P(obs | nloc, action) * P(nloc | loc, action)
        //

        float previous = 0;
        int current_loc = p.loc_history_.back();
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            float prob = 0;
            for( int label = 0; label < base_->nlabels_; ++label )
                prob += base_->probability_obs_standard(obs, new_loc, label, last_action) * p.probability(label, new_loc);
            cdf.push_back(previous + base_->probability_tr_loc(last_action, current_loc, new_loc) * prob);
            previous = cdf.back();
        }

        // normalize (i.e. calculate alpha)
        assert(cdf.back() > 0);
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            cdf[new_loc] /= cdf.back();
        }
    }

    virtual void sample_from_pi(rbpf_slam_particle_t &np,
                                const rbpf_slam_particle_t &p,
                                int last_action,
                                int obs,
                                const history_container_t &/*history_container*/,
                                mpi_slam_t * /*mpi*/,
                                int /*wid*/) const {
#ifdef DEBUG
        assert(np == p);
#endif
        calculate_cdf(p, last_action, obs, cdf_);
        int next_loc = Utils::sample_from_distribution(base_->nloc_, &cdf_[0]);
        np.loc_history_.push_back(next_loc);
        np.update_factors(last_action, obs);
    }

    virtual float importance_weight(const rbpf_slam_particle_t &, const rbpf_slam_particle_t &, int, int) const {
        return 1;
    }

    optimal_rbpf_slam_particle_t* initial_sampling(mpi_slam_t * /*mpi*/, int /*wid*/) {
        std::cout << "hola" << std::endl;
        optimal_rbpf_slam_particle_t *p = new optimal_rbpf_slam_particle_t;
        p->initial_sampling_in_place();
        std::cout << "chao" << std::endl;
        return p;
    }
};


// Helper class that pre-computes cdfs for the optimal SIR filter
template <typename PTYPE, typename BASE> struct cdf_for_optimal_sir_t {
    const BASE &base_;
    int nstates_;
    int nobs_;
    int nactions_;
    float **cdf_;

    cdf_for_optimal_sir_t(const BASE &base) : base_(base) {
        nstates_ = base_.nloc_;
        for( int loc = 0; loc < base_.nloc_; ++loc )
            nstates_ *= base_.nlabels_;
        nobs_ = base_.nlabels_;
        nactions_ = 4;
        calculate_cdf_for_pi();
    }
    ~cdf_for_optimal_sir_t() {
        for( int i = 0; i < nstates_ * nobs_ * nactions_; ++i )
            delete[] cdf_[i];
        delete[] cdf_;
    }

    void calculate_cdf_for_pi() {
        cdf_ = new float*[nstates_ * nobs_ * nactions_];

        // calculate cdfs
        for( int i = 0; i < nstates_ * nobs_ * nactions_; ++i ) {
            cdf_[i] = new float[nstates_];
            PTYPE p;
            int state_index = i % nstates_;
            int obs = (i / nstates_) % nobs_;
            int action = (i / nstates_) / nobs_;
            p.decode(state_index);
            for( int state_index2 = 0; state_index2 < nstates_; ++state_index2 ) {
                PTYPE p2;
                p2.decode(state_index2);
                cdf_[i][state_index2] = state_index2 > 0 ? cdf_[i][state_index2 - 1] : 0;
                cdf_[i][state_index2] += p.pi(p2, p, action, obs);
            }
            //cerr << "cdf[" << i << "/" << nstates_ * nobs_ * nactions_ << "][" << nstates_ - 1 << "]="
            //     << setprecision(9) << cdf_[i][nstates_ - 1] << endl;
            assert(fabs(cdf_[i][nstates_ - 1] - 1.0) < .0001);
        }
    }

    const float* pi_cdf(int state_index, int last_action, int obs) const {
        return cdf_[(last_action * nobs_ + obs) * nstates_ + state_index];
    }
};

#undef DEBUG

#endif

