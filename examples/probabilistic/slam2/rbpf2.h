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

#ifndef RBPF2_H
#define RBPF2_H

#include <cassert>
#include <string>
#include <iostream>
#include <vector>

#include "sir2.h"


template <typename PTYPE, typename BASE> struct RBPF2_t : public SIR2_t<PTYPE, BASE> {
    using tracking_t<BASE>::base_;
    using PF_t<PTYPE, BASE>::nparticles_;
    using PF_t<PTYPE, BASE>::particles_;
    using PF_t<PTYPE, BASE>::marginals_on_vars_;

    RBPF2_t(const std::string &name, const BASE &base, int nparticles)
      : SIR2_t<PTYPE, BASE>(name, base, nparticles) {
    }
    virtual ~RBPF2_t() { }

    virtual void sample_from_pi(PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        p.sample_from_pi(np, p, last_action, obs);
    }

    virtual float importance_weight(const PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        return p.importance_weight(np, p, last_action, obs);
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
#if 0
                marginals_on_vars_[var].set(p.value_for(var), marginals_on_vars_[var][p.value_for(var)] + weight);
#endif
            }
        }

#if 0
        marginals_on_vars_ = std::vector<dai::Factor>(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            marginals_on_vars_[i] = dai::Factor(dai::VarSet(dai::Var(i, nlabels_)), 0.0);
        marginals_on_vars_[nloc_] = dai::Factor(dai::VarSet(dai::Var(nloc_, nloc_)), 0.0);
        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const RBPF_particle_t &p = particles_[i].second;
            for( int loc = 0; loc < nloc_; ++loc ) {
                for( int label = 0; label < nlabels_; ++label )
                    marginals_on_vars_[loc].set(label, marginals_on_vars_[loc][label] + weight * p.probability(label, loc));
            }
            marginals_on_vars_[nloc_].set(p.current_loc(), marginals_on_vars_[nloc_][p.current_loc()] + weight);
        }
#endif
    }
};

#endif

