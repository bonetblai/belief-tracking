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

#ifndef TRACKING_H
#define TRACKING_H

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

#define EPSILON 0.000001


// General Tracking Algorithm
template <typename BASE> struct tracking_t {
    std::string name_;
    const BASE &base_;
    std::vector<std::vector<dai::Factor> > marginals_;

    tracking_t(const std::string &name, const BASE &base)
      : name_(name), base_(base) { };
    virtual ~tracking_t() { }

    virtual void initialize() = 0;
    virtual void update(int last_action, int obs) = 0;
    virtual void calculate_marginals() = 0;
    virtual void get_marginal(int var, dai::Factor &marginal) const = 0;

    void store_marginals() {
        marginals_.push_back(std::vector<dai::Factor>(base_.nvars_));
        for( int var = 0; var < base_.nvars_; ++var )
            get_marginal(var, marginals_.back()[var]);
    }

    const dai::Factor& stored_marginal(int t, int var) const {
        assert(t < int(marginals_.size()));
        const std::vector<dai::Factor> &marginals_at_time_t = marginals_[t];
        assert(var < int(marginals_at_time_t.size()));
        return marginals_at_time_t[var];
    }

    void MAP_on_var(int var, std::vector<int> &map_values, float epsilon = EPSILON) const {
        map_values.clear();
        dai::Factor marginal;
        get_marginal(var, marginal);
        float max_probability = 0;
        for( size_t value = 0; value < marginal.nrStates(); ++value ) {
            if( (marginal[value] - max_probability > epsilon) || (fabs(max_probability - marginal[value]) < epsilon) ) {
                if( marginal[value] - max_probability > epsilon )
                    map_values.clear();
                max_probability = marginal[value];
                map_values.push_back(value);
            }
        }
    }
};

#endif

