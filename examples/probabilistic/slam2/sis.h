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


// Sequential Importance Sampling (SIS) Particle Filter
template <typename PTYPE, typename BASE> struct SIS_t : public PF_t<PTYPE, BASE> {
    using tracking_t<BASE>::base_;
    using PF_t<PTYPE, BASE>::nparticles_;
    using PF_t<PTYPE, BASE>::particles_;
    using PF_t<PTYPE, BASE>::marginals_on_vars_;

    SIS_t(const std::string &name, const BASE &base, int nparticles)
      : PF_t<PTYPE, BASE>(name, base, nparticles) {
    }
    virtual ~SIS_t() { }

    virtual void initialize() {
        particles_ = std::vector<std::pair<float, PTYPE> >(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1;
            particles_[i].second.initial_sampling();
        }
    }

    virtual void update(int last_action, int obs) {
        for( int i = 0; i < nparticles_; ++i ) {
            float &weight = particles_[i].first;
            PTYPE &p = particles_[i].second;
            p.update(weight, last_action, obs); // changes "weight" which is passed as ref
        }
    }

    virtual void calculate_marginals() {
#if 0
        marginals_on_vars_ = std::vector<dai::Factor>(nloc_ + 1);
        for( int i = 0; i < nloc_; ++i )
            marginals_on_vars_[i] = dai::Factor(dai::VarSet(dai::Var(i, nlabels_)), 0.0);
        marginals_on_vars_[nloc_] = dai::Factor(dai::VarSet(dai::Var(nloc_, nloc_)), 0.0);

        float total_mass = 0;
        for( int i = 0; i < nparticles_; ++i )
            total_mass += particles_[i].first;

        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const std::vector<int> &p = particles_[i].second;
            for( int j = 0; j <= nloc_; ++j )
                marginals_on_vars_[j].set(p[j], marginals_on_vars_[j][p[j]] + weight / total_mass);
        }
#endif
    }

    virtual void get_marginal(int var, dai::Factor &marginal) const {
        marginal = marginals_on_vars_[var];
    }
};

#endif

