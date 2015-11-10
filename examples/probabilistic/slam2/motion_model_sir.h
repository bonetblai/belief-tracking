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

#ifndef MOTION_MODEL_SIR_H
#define MOTION_MODEL_SIR_H

#include <cassert>
#include <string>

#include "sir.h"

// SIR filter with proposal distribution given by motion model
template <typename PTYPE, typename BASE> struct motion_model_SIR_t : public SIR_t<PTYPE, BASE> {
    motion_model_SIR_t(const std::string &name, const BASE &base, int nparticles, bool do_resampling, bool do_stochastic_universal_sampling)
      : SIR_t<PTYPE, BASE>(name, base, nparticles, do_resampling, do_stochastic_universal_sampling) {
    }
    virtual ~motion_model_SIR_t() { }

    virtual void sample_from_pi(PTYPE &np, const PTYPE &p, int last_action, int obs, const history_container_t &/*history_container*/, int /*wid*/) const {
        p.sample_from_pi(np, last_action, obs);
    }
    virtual float importance_weight(const PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        return p.importance_weight(np, last_action, obs);
    }
};

#endif

