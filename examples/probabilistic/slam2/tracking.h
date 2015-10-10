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

#define EPSILON 1e-4

// Repositoy of marginals for each time step
class repository_t : public std::vector<float*> { };

// General Tracking Algorithm
template <typename BASE> struct tracking_t {
    std::string name_;
    const BASE &base_;

    tracking_t(const std::string &name, const BASE &base)
      : name_(name), base_(base) {
    }
    virtual ~tracking_t() { }

    virtual void initialize() = 0;
    virtual void update(int last_action, int obs) = 0;
    virtual void calculate_marginals() = 0;
    virtual void get_marginal(int var, dai::Factor &marginal) const = 0;
    virtual float* get_marginal(int var, float *ptr) const = 0;
    virtual std::string id() const = 0;

    bool verify_marginals(const float *marginals) const {
        int good = true;
        for( int var = 0; var < base_.nvars_; ++var ) {
            float sum = 0;
            for( int value = 0; value < base_.variable_size(var); ++value )
                sum += marginals[base_.variable_offset(var) + value];
            if( fabs(sum - 1.0) >= EPSILON ) {
                good = false;
                std::cout << "warning: tracking_t::verify_marginals(): sum=" << sum << std::endl;
                //assert(fabs(sum - 1.0) < EPSILON);
            }
        }
        return good;
    }

    void store_marginals(repository_t &repository) const {
        float *marginals = new float[base_.marginals_size_];
        float *ptr = marginals;
        for( int var = 0; var < base_.nvars_; ++var )
            ptr = get_marginal(var, ptr);
        verify_marginals(marginals);
        repository.push_back(marginals);
    }

    const float* stored_marginal(const repository_t &repository, int t, int var) const {
        assert(t < int(repository.size()));
        const float *marginals = repository[t];
        return &marginals[base_.variable_offset(var)];
    }

    void clean(repository_t &repository) const {
        while( !repository.empty() ) {
            delete repository.back();
            repository.pop_back();
        }
    }

    void MAP_on_var(const repository_t &repository, int var, std::vector<int> &map_values, float epsilon = EPSILON) const {
        assert(!repository.empty());
        const float *marginals = repository.back();
        const float *marginal = &marginals[base_.variable_offset(var)];
        map_values.clear();
        float max_probability = 0;
        for( int value = 0; value < base_.variable_size(var); ++value ) {
            if( (marginal[value] - max_probability > epsilon) || (fabs(max_probability - marginal[value]) < epsilon) ) {
                if( marginal[value] - max_probability > epsilon )
                    map_values.clear();
                max_probability = marginal[value];
                map_values.push_back(value);
            }
        }
    }

    void print_marginals(std::ostream &os, const repository_t &repository) const {
        assert(!repository.empty());
        const float *last_marginals = repository.back();
        for( int var = 0; var < base_.nvars_; ++var ) {
            const float *marginal = &last_marginals[base_.variable_offset(var)];
            os << "var " << var << " [dsz=" << base_.variable_size(var) << "]:";
            for( int value = 0; value < base_.variable_size(var); ++value )
                os << " " << marginal[value];
            os << std::endl;
        }
    }
};

#endif

