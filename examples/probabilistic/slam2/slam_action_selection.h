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

#ifndef NAIVE_ACTION_SELECTION_H
#define NAIVE_ACTION_SELECTION_H

#include <cassert>
#include <math.h>
#include "action_selection.h"
#include "cellmap.h"

struct naive_action_selection_t : public action_selection_t<cellmap_t> {
    float p_;
    naive_action_selection_t(const cellmap_t &cellmap, float p)
      : action_selection_t<cellmap_t>(cellmap), p_(p) { }
    virtual ~naive_action_selection_t() { }
    virtual int select_action(const tracking_t<cellmap_t> */*tracking*/) const {
        int action = drand48() > p_ ? cellmap_t::right : cellmap_t::left;
        return action;
    }
};

#endif

