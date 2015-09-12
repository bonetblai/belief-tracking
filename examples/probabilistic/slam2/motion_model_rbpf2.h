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

#ifndef MOTION_MODEL_RBPF2_H
#define MOTION_MODEL_RBPF2_H

#include <cassert>
#include <string>

#include "rbpf2.h"

// RBPF filter with proposal distribution given by motion model
template <typename PTYPE, typename BASE> struct motion_model_RBPF2_t : public RBPF2_t<PTYPE, BASE> {
    motion_model_RBPF2_t(const std::string &name, const BASE &base, int nparticles)
      : RBPF2_t<PTYPE, BASE>(name, base, nparticles) {
    }
    virtual ~motion_model_RBPF2_t() { }

    virtual void sample_from_pi(PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        p.sample_from_pi(np, p, last_action, obs);
    }
    virtual float importance_weight(const PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        return p.importance_weight(np, p, last_action, obs);
    }
};

#endif

