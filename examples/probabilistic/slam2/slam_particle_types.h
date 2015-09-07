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
#include <math.h>

#include <dai/alldai.h>

// Generic particle for the color-tile SLAM problem
struct slam_particle_t {
    int current_loc_;
    std::vector<int> map_;
    static const cellmap_t *base_;

    void initial_sampling() {
        map_ = std::vector<int>(base_->nloc_);
        for( int i = 0; i < base_->nloc_; ++i )
            map_[i] = lrand48() % base_->nlabels_;
        current_loc_ = base_->initial_loc_;
    }
};

// Particle for the SIS filter
struct sis_slam_particle_t : public slam_particle_t {
    void update(float &weight, int last_action, int obs) {
        int new_loc = base_->sample_loc(current_loc_, last_action);
        weight *= base_->obs_probability(obs, current_loc_, last_action); // CHECK: IS THIS OK? SHOULD IT BE NEW_LOC INSTEAD?
        current_loc_ = new_loc;
    }
};

// Particle for the motion model SIR2 filter
struct motion_model_sir2_slam_particle_t : public slam_particle_t {
    void sample_from_pi(motion_model_sir2_slam_particle_t &np, const motion_model_sir2_slam_particle_t &p, int last_action, int /*obs*/) const {
        np = p;
        np.current_loc_ = base_->sample_loc(p.current_loc_, last_action);
    }
    float importance_weight(const motion_model_sir2_slam_particle_t &np, const motion_model_sir2_slam_particle_t &/*p*/, int last_action, int obs) const {
        return base_->obs_probability(obs, np.current_loc_, np.map_, last_action);
    }
};

// Particle for the optimal SIR2 filter
struct optimal_sir2_slam_particle_t : public slam_particle_t {
    int encode() const {
        int state_index = 0;
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            state_index *= base_->nlabels_;
            state_index += map_[loc];
        }
        state_index = state_index * base_->nloc_ + current_loc_;
        return state_index;
    }

    void decode(int state_index) {
        map_ = std::vector<int>(base_->nloc_, 0);
        current_loc_ = state_index % base_->nloc_;
        state_index /= base_->nloc_;
        for( int loc = base_->nloc_ - 1; loc >= 0; --loc ) {
            map_[loc] = state_index % base_->nlabels_;
            state_index /= base_->nlabels_;
        }
    }

    float pi(const optimal_sir2_slam_particle_t &np, const optimal_sir2_slam_particle_t &p, int last_action, int obs) const {
        // np has probability:
        //   P(np|p,last_action,obs) = P(np,obs|p,last_action) / P(obs|p,last_action)
        //                           = P(obs|np,p,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        //                           = P(obs|np,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        if( np.map_ == p.map_ ) {
            float prob = base_->obs_probability(obs, np.current_loc_, np.map_, last_action);
            prob *= base_->loc_probability(last_action, p.current_loc_, np.current_loc_);
            return prob / importance_weight(np, p, last_action, obs);
        } else {
            return 0;
        }
    }

    float importance_weight(const optimal_sir2_slam_particle_t &/*np*/, const optimal_sir2_slam_particle_t &p, int last_action, int obs) const {
        // weight = P(obs|np,last_action) * P(np|p,last_action) / P(np|p,last_action,obs)
        //        = P(obs|p,last_action) [see above derivation in pi(..)]
        //        = SUM P(np,obs|p,last_action)
        //        = SUM P(obs|np,p,last_action) P(np|p,last_action)
        //        = SUM P(obs|np,last_action) P(np|p,last_action)
        // since P(np|p,last_action) = 0 if p and np have different labels,
        // we can simplify the sum over locations instead of maps
        float weight = 0;
        int loc = p.current_loc_;
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc )
            weight += base_->obs_probability(obs, new_loc, p.map_, last_action) * base_->loc_probability(last_action, loc, new_loc);
        return weight;
    }
};

// Particle for the Rao-Blackwellised filter
struct rbpf_slam_particle_t : public slam_particle_t {
    std::vector<dai::Factor> factors_;

    void initial_sampling() {
        //base_particle_t::initial_sampling();
        current_loc_ = base_->initial_loc_;
        factors_ = std::vector<dai::Factor>(base_->nloc_, dai::Factor(dai::VarSet(dai::Var(0, base_->nlabels_)), 1.0 / float(base_->nlabels_)));
    }
 
    void update(int last_action, int obs) {
        assert(current_loc_ < int(factors_.size()));
        dai::Factor &factor = factors_[current_loc_];
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

    void sample_from_pi(rbpf_slam_particle_t &np, const rbpf_slam_particle_t &p, int last_action, int obs) const {
        np = p;
        np.current_loc_ = base_->sample_loc(p.current_loc_, last_action);
        np.update(last_action, obs);
    }

    float importance_weight(const rbpf_slam_particle_t &np, const rbpf_slam_particle_t &p, int last_action, int obs) const {
        float prob = 0;
        for( int label = 0; label < base_->nlabels_; ++label ) // marginalize over possible labels at current loc
            prob += base_->obs_probability(obs, label, last_action) * p.probability(label, np.current_loc_);
        return prob;
    }

    const rbpf_slam_particle_t& operator=(const rbpf_slam_particle_t &p) {
        current_loc_ = p.current_loc_;
        factors_ = p.factors_;
        return *this;
    }
};

#endif

