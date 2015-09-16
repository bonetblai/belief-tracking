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

#ifndef SLAM_PARTICLE_TYPES_H
#define SLAM_PARTICLE_TYPES_H

#include <cassert>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <stdlib.h>

#include "utils.h"

#include <dai/alldai.h>

// Generic particle for the color-tile SLAM problem
struct base_particle_t {
    static const cellmap_t *base_;
};

struct slam_particle_t : public base_particle_t {
    int current_loc_;
    std::vector<int> labels_;

    void initial_sampling() {
        labels_ = std::vector<int>(base_->nloc_);
        for( int i = 0; i < base_->nloc_; ++i )
            labels_[i] = lrand48() % base_->nlabels_;
        current_loc_ = base_->initial_loc_;
    }

    int value_for(int var) const {
        return var == base_->nvars_ - 1 ? current_loc_ : labels_[var];
    }
};

// Particle for the SIS filter
struct sis_slam_particle_t : public slam_particle_t {
    void update(float &weight, int last_action, int obs) {
        int new_loc = base_->sample_loc(current_loc_, last_action);
        weight *= base_->obs_probability(obs, labels_[new_loc], last_action);
        current_loc_ = new_loc;
    }
};

// Particle for the motion model SIR filter (verified: 09/12/2015)
struct motion_model_sir_slam_particle_t : public slam_particle_t {
    void sample_from_pi(motion_model_sir_slam_particle_t &np, const motion_model_sir_slam_particle_t &p, int last_action, int /*obs*/) const {
        np = p;
        np.current_loc_ = base_->sample_loc(p.current_loc_, last_action);
    }

    float importance_weight(const motion_model_sir_slam_particle_t &np, const motion_model_sir_slam_particle_t &/*p*/, int last_action, int obs) const {
        return base_->obs_probability(obs, np.current_loc_, np.labels_, last_action);
    }
};

// Particle for the optimal SIR filter (verified: 09/12/2015)
struct optimal_sir_slam_particle_t : public slam_particle_t {
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

    float pi(const optimal_sir_slam_particle_t &np, const optimal_sir_slam_particle_t &p, int last_action, int obs) const {
        // pi(np|p,last_action,obs) = P(np|p,last_action,obs)
        //                          = P(np,obs|p,last_action) / P(obs|p,last_action)
        //                          = P(obs|np,p,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        //                          = P(obs|np,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        if( np.labels_ == p.labels_ ) {
            float prob = base_->obs_probability(obs, np.current_loc_, np.labels_, last_action);
            prob *= base_->loc_probability(last_action, p.current_loc_, np.current_loc_);
            return prob / importance_weight(np, p, last_action, obs);
        } else {
            return 0;
        }
    }

    float importance_weight(const optimal_sir_slam_particle_t &/*np*/, const optimal_sir_slam_particle_t &p, int last_action, int obs) const {
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
            weight += base_->obs_probability(obs, new_loc, p.labels_, last_action) * base_->loc_probability(last_action, loc, new_loc);
        return weight;
    }
};

// Abstract Particle for the Rao-Blackwellised filter
struct rbpf_slam_particle_t : public base_particle_t {
    std::vector<int> loc_history_;
    std::vector<dai::Factor> factors_;

    void initial_sampling() {
        loc_history_.push_back(base_->initial_loc_);
        factors_ = std::vector<dai::Factor>(base_->nloc_, dai::Factor(dai::VarSet(dai::Var(0, base_->nlabels_)), 1.0 / float(base_->nlabels_)));
    }

    void update_factors(int last_action, int obs) {
        int current_loc = loc_history_.back();
        assert(current_loc < int(factors_.size()));
        dai::Factor &factor = factors_[current_loc];
        assert(base_->nlabels_ == int(factor.nrStates()));
        float total_mass = 0.0;
        for( int label = 0; label < base_->nlabels_; ++label ) {
            factor.set(label, factor[label] * base_->obs_probability(obs, label, last_action));
            total_mass += factor[label];
        }
        assert(total_mass > 0);
        factor /= total_mass;
    }

    float probability(int label, int loc) const {
        assert(loc < int(factors_.size()));
        assert(label < int(factors_[loc].nrStates()));
        return factors_[loc][label];
    }

    void update_marginal(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        int current_loc = loc_history_.back();
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            for( int label = 0; label < base_->nlabels_; ++label )
                marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * probability(label, loc));
        }
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
    }

    int value_for(int /*var*/) const { return -1; }

    virtual void sample_from_pi(rbpf_slam_particle_t &np, const rbpf_slam_particle_t &p, int last_action, int obs) const = 0;
    virtual float importance_weight(const rbpf_slam_particle_t &np, const rbpf_slam_particle_t &p, int last_action, int obs) const = 0;
};

// Particle for the motion model RBPF filter (verified: 09/12/2015)
struct motion_model_rbpf_slam_particle_t : public rbpf_slam_particle_t {
    virtual void sample_from_pi(rbpf_slam_particle_t &np, const rbpf_slam_particle_t &p, int last_action, int obs) const {
        np = p;
        int next_loc = base_->sample_loc(p.loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        np.update_factors(last_action, obs);
    }

    virtual float importance_weight(const rbpf_slam_particle_t &np, const rbpf_slam_particle_t &p, int last_action, int obs) const {
        float weight = 0;
        for( int label = 0; label < base_->nlabels_; ++label ) // marginalize over possible labels at current loc
            weight += base_->obs_probability(obs, label, last_action) * p.probability(label, np.loc_history_.back());
        return weight;
    }
};

// Particle for the optimal RBPF filter (verified: 09/12/2015)
struct optimal_rbpf_slam_particle_t : public rbpf_slam_particle_t {
    mutable std::vector<float> cdf_;

    void calculate_cdf(const rbpf_slam_particle_t &p, int last_action, int obs) const {
        cdf_.clear();
        cdf_.reserve(base_->nloc_);

        int current_loc = p.loc_history_.back();
        float previous = 0;

        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            float prob = 0;
            for( int label = 0; label < base_->nlabels_; ++label )
                prob += base_->obs_probability(obs, label, last_action) * p.probability(label, new_loc);
            cdf_.push_back(previous + base_->loc_probability(last_action, current_loc, new_loc) * prob);
            previous = cdf_.back();
        }

        // normalize
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            cdf_[new_loc] /= cdf_.back();
        }
        assert(cdf_.back() == 1.0);
    }

    virtual void sample_from_pi(rbpf_slam_particle_t &np, const rbpf_slam_particle_t &p, int last_action, int obs) const {
        // sample new_loc w.p. P(new_loc|curr_loc,last_action) * SUM_c P(obs|new_loc,Label[new_loc]=c) P(c|new_loc)
        np = p;
        calculate_cdf(p, last_action, obs);
        int next_loc = Utils::sample_from_distribution(base_->nloc_, &cdf_[0]);
        np.loc_history_.push_back(next_loc);
        np.update_factors(last_action, obs);
    }

    virtual float importance_weight(const rbpf_slam_particle_t &/*np*/, const rbpf_slam_particle_t &p, int last_action, int obs) const {
        int current_loc = p.loc_history_.back();
        float weight = 0;
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            float prob = 0;
            for( int label = 0; label < base_->nlabels_; ++label )
                prob += base_->obs_probability(obs, label, last_action) * p.probability(label, new_loc);
            weight += base_->loc_probability(last_action, current_loc, new_loc) * prob;
        }
        return weight;
    }
};


// Helper class that pre-computes cdfs for the optimal SIR filter (verified: 09/12/2015)
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

#endif

