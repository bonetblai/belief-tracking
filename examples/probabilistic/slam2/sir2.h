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

#ifndef SIR2_H
#define SIR2_H

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

#include "particle_filter.h"


// Generic Sequential Importance Resampling (SIR2) Particle Filter
template <typename PTYPE, typename BASE> struct SIR2_t : public PF_t<PTYPE, BASE> {
    using tracking_t<BASE>::base_;
    using PF_t<PTYPE, BASE>::nparticles_;
    using PF_t<PTYPE, BASE>::particles_;
    using PF_t<PTYPE, BASE>::marginals_on_vars_;

    bool do_stochastic_universal_sampling_;
    std::vector<std::pair<int, int> > history_;

    SIR2_t(const std::string &name, const BASE &base, int nparticles, bool do_stochastic_universal_sampling = false)
      : PF_t<PTYPE, BASE>(name, base, nparticles), do_stochastic_universal_sampling_(do_stochastic_universal_sampling) {
    }
    virtual ~SIR2_t() { }

    virtual void sample_from_pi(PTYPE &np, const PTYPE &p, int last_action, int obs) const = 0;
    virtual float importance_weight(const PTYPE &np, const PTYPE &p, int last_action, int obs) const = 0;

    virtual void initialize() {
        particles_ = std::vector<std::pair<float, PTYPE> >(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1.0 / float(nparticles_);
            particles_[i].second.initial_sampling();
        }
    }

    virtual void update(int last_action, int obs) {
        std::vector<int> indices;
        if( do_stochastic_universal_sampling_ )
            PF_t<PTYPE, BASE>::stochastic_universal_sampling(nparticles_, indices);
        else
            PF_t<PTYPE, BASE>::stochastic_sampling(nparticles_, indices);
        assert(indices.size() == size_t(nparticles_));

        float total_mass = 0.0;
        std::vector<std::pair<float, PTYPE> > new_particles;
        new_particles.reserve(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            int index = indices[i];
            const PTYPE &p = particles_[index].second;
            PTYPE np;
            sample_from_pi(np, p, last_action, obs);
            float weight = importance_weight(np, p, last_action, obs);
            total_mass += weight;
            new_particles.push_back(std::make_pair(weight, np));
        }
        for( int i = 0; i < nparticles_; ++i )
            new_particles[i].first /= total_mass;
        particles_ = new_particles;

        history_.push_back(std::make_pair(last_action, obs));
    }

    virtual void calculate_marginals() {
        // initialize marginals
        marginals_on_vars_ = std::vector<dai::Factor>(base_.nvars_);
        for( int var = 0; var < base_.nvars_; ++var ) {
            marginals_on_vars_[var] = dai::Factor(dai::VarSet(dai::Var(var, base_.domain_size(var))), 0.0);
        }

        // aggregate info from particles into marginals
        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const PTYPE &p = particles_[i].second;
            for( int var = 0; var < base_.nvars_; ++var ) {
                marginals_on_vars_[var].set(p.value_for(var), marginals_on_vars_[var][p.value_for(var)] + weight);
            }
        }
    }

    virtual void get_marginal(int var, dai::Factor &marginal) const {
        marginal = marginals_on_vars_[var];
    }
};

#endif // David: 305 577 6954

