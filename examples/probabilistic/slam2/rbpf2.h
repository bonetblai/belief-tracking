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

#if 0
    virtual void sample_from_pi(PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        p.sample_from_pi(np, p, last_action, obs);
    }

    virtual float importance_weight(const PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        return p.importance_weight(np, p, last_action, obs);
    }
#endif

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
            p.update_marginal(weight, marginals_on_vars_);
        }
    }
};

#endif

