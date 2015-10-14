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

#ifndef OPTIMAL_SIR_H
#define OPTIMAL_SIR_H

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


// SIR filter with proposal distribution given by optimal choice
template <typename PTYPE, typename BASE, typename CDF> struct optimal_SIR_t : public SIR_t<PTYPE, BASE> {
    using PF_t<PTYPE, BASE>::sample_from_distribution;

    const CDF &cdf_;

    optimal_SIR_t(const std::string &name, const BASE &base, const CDF &cdf, int nparticles)
      : SIR_t<PTYPE, BASE>(name, base, nparticles), cdf_(cdf) {
    }
    virtual ~optimal_SIR_t() { }

    virtual void sample_from_pi(PTYPE &np, const PTYPE &p, int last_action, int obs, int /*pindex*/) const {
        const float *dist = cdf_.pi_cdf(p.encode(), last_action, obs);
        int code = sample_from_distribution(cdf_.nstates_, dist);
        np.decode(code);
    }

    virtual float importance_weight(const PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        return p.importance_weight(np, p, last_action, obs);
    }
};

#endif

