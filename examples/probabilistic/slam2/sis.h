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

#ifndef SIS_H
#define SIS_H

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

// TODO: use multiplicities for particles


// Sequential Importance Sampling (SIS) Particle Filter
template <typename PTYPE, typename BASE> struct SIS_t : public PF_t<PTYPE, BASE> {
    using tracking_t<BASE>::base_;
    using PF_t<PTYPE, BASE>::nparticles_;
    using PF_t<PTYPE, BASE>::particles_;
    using PF_t<PTYPE, BASE>::marginals_on_vars_;
    typedef typename PF_t<PTYPE, BASE>::particle_t particle_t;

    SIS_t(const std::string &name, const BASE &base, const std::multimap<std::string, std::string> &parameters)
      : PF_t<PTYPE, BASE>(name, base) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("nparticles");
        if( it != parameters.end() )
            PF_t<PTYPE, BASE>::set_nparticles(strtol(it->second.c_str(), 0, 0));
    }
    virtual ~SIS_t() { /* who deletes particles? */ }

    virtual void clear() {
        PF_t<PTYPE, BASE>::clear_particles();
    }

    virtual void initialize() {
        PTYPE sampler;
        particles_.reserve(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            PTYPE *p = sampler.initial_sampling();
            particles_.push_back(particle_t(1, p));
        }
    }

    virtual void update(int last_action, int obs) {
        for( int i = 0; i < nparticles_; ++i ) {
            float &weight = particles_[i].weight_;
            PTYPE &p = *particles_[i].p_;
            p.update(weight, last_action, obs); // changes "weight" which is passed as ref
        }
    }

    virtual void calculate_marginals() {
        // initialize marginals
        marginals_on_vars_ = std::vector<dai::Factor>(base_.nvars_);
        for( int var = 0; var < base_.nvars_; ++var ) {
            marginals_on_vars_[var] = dai::Factor(dai::VarSet(dai::Var(var, base_.variable_size(var))), 0.0);
        }

        float total_mass = 0;
        for( int i = 0; i < nparticles_; ++i )
            total_mass += particles_[i].weight_;

        // aggregate info from particles into marginals
        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].weight_;
            const PTYPE &p = *particles_[i].p_;
            for( int var = 0; var < base_.nvars_; ++var ) {
                marginals_on_vars_[var].set(p.value_for(var), marginals_on_vars_[var][p.value_for(var)] + weight / total_mass);
            }
        }
    }

    virtual void get_marginal(int var, dai::Factor &marginal) const {
        marginal = marginals_on_vars_[var];
    }

    virtual float* get_marginal(int var, float *ptr) const {
        const dai::Factor &factor = marginals_on_vars_[var];
        for( int value = 0; value < int(factor.nrStates()); ++value )
            *ptr++ = factor[value];
        return ptr;
    }

    virtual std::string id() const {
        std::string id_str;
        id_str = std::string("PF(type=SIS") +
          std::string(",ptype=") + PTYPE::type() +
          std::string(",nparticles=") + std::to_string((long long)nparticles_) +
          ")";
        return id_str;
    }
};

#endif

