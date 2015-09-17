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

#ifndef SLAM2_PARTICLE_TYPES_H
#define SLAM2_PARTICLE_TYPES_H

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
#include "slam_particle_types.h"
#include "utils.h"

// Abstract Particle for the 2nd Rao-Blackwellised filter
struct rbpf_slam2_particle_t : public base_particle_t {
    std::vector<int> loc_history_;

    // variables, factors, and centers
    std::vector<dai::Var> variables_;
    std::vector<dai::Factor> factors_;
    std::vector<int> centers_;

    rbpf_slam2_particle_t() {
        assert(base_ != 0);
        assert(base_->nlabels_ == 2);

        // create binary variables for each cell in the grid
        int nloc = base_->nloc_;
        variables_ = std::vector<dai::Var>(nloc);
        for( int loc = 0; loc < nloc; ++loc )
            variables_[loc] = dai::Var(loc, 2);

        // create one factor for each location. The variables
        // in the factor are the variables for the location
        // surrounding the factor, including the variable
        // for the "center" location. Also set up the center
        // for each factor.
        centers_ = std::vector<int>(nloc);
        factors_ = std::vector<dai::Factor>(nloc);
        for( int loc = 0; loc < nloc; ++loc ) {
            int row = loc / base_->ncols_, col = loc % base_->ncols_;
            std::vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= base_->nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= base_->ncols_) ) continue;
                    if( (dr == 0) && (dc == 0) ) centers_[loc] = vars.size();
                    vars.push_back(variables_[nr * base_->ncols_ + nc]);
                }
            }
            dai::VarSet varset(vars.begin(), vars.end());
            factors_[loc] = dai::Factor(varset);
        }
    }
    virtual ~rbpf_slam2_particle_t() { }

    void initial_sampling() {
        assert(base_->nlabels_ == 2);

        // set initial history and reset factors for locations
        loc_history_.push_back(base_->initial_loc_);
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / (1 << factor.vars().size());
            for( int i = 0; i < (1 << factor.vars().size()); ++i )
                factor.set(i, p);
            std::cout << "factor-size: loc=" << loc << ":" << coord_t(loc) << ", sz=" << factor.vars().size() << std::endl;
        }
    }

    void update_factors(int last_action, int obs) {
#if 0
        int current_loc = loc_history_.back();
        assert(current_loc < int(factors_.size()));
        dai::Factor &factor = factors_[current_loc];
        assert(base_->nlabels_ == int(factor.nrStates()));
        float total_mass = 0.0;
        for( int label = 0; label < base_->nlabels_; ++label ) {
            factor.set(label, factor[label] * base_->probability_obs(obs, label, last_action));
            total_mass += factor[label];
        }
        assert(total_mass > 0);
        factor /= total_mass;
#endif
    }

#if 0
    float probability(int label, int loc) const {
        assert(loc < int(factors_.size()));
        assert(label < int(factors_[loc].nrStates()));
        return factors_[loc][label];
    }
#endif

    void update_marginal(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
#if 0
        int current_loc = loc_history_.back();
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            for( int label = 0; label < base_->nlabels_; ++label )
                marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * probability(label, loc));
        }
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
#endif
    }

    int value_for(int /*var*/) const { return -1; }

    virtual void sample_from_pi(rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const = 0;
    virtual float importance_weight(const rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const = 0;
};

// Particle for the motion model RBPF filter (slam2)
struct motion_model_rbpf_slam2_particle_t : public rbpf_slam2_particle_t {
    virtual void sample_from_pi(rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        np = p;
        int next_loc = base_->sample_loc(p.loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        np.update_factors(last_action, obs);
    }

    virtual float importance_weight(const rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        float weight = 0;
        int np_current_loc = np.loc_history_.back();
        const dai::Factor &p_marginal = p.factors_[np_current_loc];
        for( int slabels = 0; slabels < int(p_marginal.nrStates()); ++slabels )
            weight += p_marginal[slabels] * base_->probability_obs_special(obs, np_current_loc, slabels, last_action);
        return weight;
    }
};

#if 0
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
                prob += base_->probability_obs(obs, label, last_action) * p.probability(label, new_loc);
            cdf_.push_back(previous + base_->probability_tr_loc(last_action, current_loc, new_loc) * prob);
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
                prob += base_->probability_obs(obs, label, last_action) * p.probability(label, new_loc);
            weight += base_->probability_tr_loc(last_action, current_loc, new_loc) * prob;
        }
        return weight;
    }
};
#endif

#endif

