/*
 *  Copyright (C) 2016 Universidad Simon Bolivar
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

#ifndef SLAM_CACHE_H
#define SLAM_CACHE_H

#include <cassert>
#include <iostream>
#include <map>
#include <vector>

#include <dai/alldai.h>
#include "libdai_cache.h"

//#define DEBUG

namespace SLAM {

class cache_t : public dai::cache_t {
  protected:
    static std::vector<std::vector<int> > slabels_;

    static void compute_basic_elements(int nrows, int ncols) {
        dai::cache_t::compute_basic_elements(nrows, ncols);
    }
    static void compute_cache_for_states(int nrows, int ncols) {
        dai::cache_t::compute_cache_for_states(nrows, ncols);
    }

  public:
    cache_t() { }
    ~cache_t() { }

    static void finalize() {
        dai::cache_t::finalize();
    }

    static int get_slabels(int loc, const dai::VarSet &varset, int value, int var_offset(int, int)) {
        assert((loc >= 0) && (loc < num_locs_));
        assert((value >= 0) && (value < int(varset.nrStates())));

        // allocate cache if this is first call
        if( slabels_.empty() )
            slabels_ = std::vector<std::vector<int> >(num_locs_);
        assert(loc < int(slabels_.size()));
        if( slabels_[loc].empty() )
            slabels_[loc] = std::vector<int>(varset.nrStates(), -1);
        assert(value < int(slabels_[loc].size()));

        // check whether there is a valid entry in cache
        const std::vector<int> &slabels_for_loc = slabels_[loc];
        if( slabels_for_loc[value] != -1 )
            return slabels_for_loc[value];

#ifdef DEBUG
        std::cout << "get_slabels(loc=" << loc << ":" << coord_t(loc) << ", value=" << value << "):" << std::endl;
#endif

        // this is the first time that we access (beam,value)
        // compute the correct value and cache it for later use
        int slabels = 0;
        const std::map<dai::Var, size_t> &state = dai::cache_t::state(loc, value);
        for( dai::VarSet::const_iterator it = varset.begin(); it != varset.end(); ++it ) {
            const dai::Var &var = *it;
            std::map<dai::Var, size_t>::const_iterator jt = state.find(var);
            assert(jt != state.end());
            size_t var_value = jt->second;
            assert(var_value < 2); // because it is a binary variable
            if( var_value ) {
                int var_id = it->label();
                int var_off = var_offset(loc, var_id);
#ifdef DEBUG
                std::cout << "    [var_id=" << var_id << ":" << coord_t(var_id)
                          << ", var_value=1, off=" << var_off << "]"
                          << std::endl;
#endif
                slabels += (1 << var_off);
            }
        }
#ifdef DEBUG
        std::cout << "    bits=|";
        print_bits(std::cout, slabels, 9);
        std::cout << "| (" << slabels << ")" << std::endl;
#endif

        // cache it and return
        slabels_[loc][value] = slabels;
        return slabels;
    }
};

class cache_with_location_t : public dai::cache_with_location_t {
  protected:
    static std::vector<std::vector<int> > slabels_;
    static std::vector<std::vector<int> > patched_values_;;

    static void compute_basic_elements(int nrows, int ncols) {
        dai::cache_with_location_t::compute_basic_elements(nrows, ncols);
    }
    static void compute_cache_for_states(int nrows, int ncols) {
        dai::cache_with_location_t::compute_cache_for_states(nrows, ncols);
    }

  public:
    cache_with_location_t() { }
    ~cache_with_location_t() { }

    static void finalize() {
        dai::cache_with_location_t::finalize();
    }

    static int get_slabels(int loc, const dai::VarSet &varset, int value, int var_offset(int, int)) {
        assert((loc >= 0) && (loc < num_locs_));
        assert((value >= 0) && (value < int(varset.nrStates())));

        // allocate cache if this is first call
        if( slabels_.empty() )
            slabels_ = std::vector<std::vector<int> >(num_locs_);
        assert(loc < int(slabels_.size()));
        if( slabels_[loc].empty() )
            slabels_[loc] = std::vector<int>(varset.nrStates(), -1);
        assert(value < int(slabels_[loc].size()));

        // check whether there is a valid entry in cache
        const std::vector<int> &slabels_for_loc = slabels_[loc];
        if( slabels_for_loc[value] != -1 )
            return slabels_for_loc[value];

#ifdef DEBUG
        std::cout << "get_slabels(loc=" << loc << ":" << coord_t(loc) << ", vars=" << varset << ", value=" << value << "):" << std::endl;
#endif

        // this is the first time that we access (beam,value)
        // compute the correct value and cache it for later use
        int slabels = 0;
        const std::map<dai::Var, size_t> &state = dai::cache_with_location_t::state(loc, value);
        for( dai::VarSet::const_iterator it = varset.begin(); it != varset.end(); ++it ) {
            if( *it == dai::cache_with_location_t::variable(num_locs_) ) continue; // skip location variable
            const dai::Var &var = *it;
            assert(state.find(var) != state.end());
            size_t var_value = state.at(var);
            assert(var_value < 2); // because it is a binary variable
            if( var_value == 1 ) {
                int var_id = it->label();
                int var_off = var_offset(loc, var_id);
#ifdef DEBUG
                std::cout << "    [var_id=" << var_id << ":" << coord_t(var_id)
                          << ", var_value=1, off=" << var_off << "]"
                          << std::endl;
#endif
                slabels += (1 << var_off);
            }
        }
#ifdef DEBUG
        std::cout << "    bits=|";
        print_bits(std::cout, slabels, 9);
        std::cout << "| (" << slabels << ")" << std::endl;
#endif

        // cache it and return
        slabels_[loc][value] = slabels;
        return slabels;
    }

    static int replace_loc_in_value(int loc, const dai::VarSet &varset, int value, int old_loc, int new_loc) {
        assert((loc >= 0) && (loc < num_locs_));
        assert((new_loc >= 0) && (new_loc < num_locs_));
        assert((value >= 0) && (value < int(varset.nrStates())));

        // allocate cache if this is first call
        if( patched_values_.empty() )
            patched_values_ = std::vector<std::vector<int> >(num_locs_);
        assert(loc < int(patched_values_.size()));
        if( patched_values_[loc].empty() )
            patched_values_[loc] = std::vector<int>(varset.nrStates(), -1);
        assert(value < int(patched_values_[loc].size()));

        // check whether there is a valid entry in cache
        const std::vector<int> &patched_values_for_loc = patched_values_[loc];
        if( patched_values_for_loc[value] != -1 )
            return patched_values_for_loc[value];

#ifdef DEBUG
        std::cout << "replace_loc_in_value(loc=" << loc << ":" << coord_t(loc)
                  << ", value=" << value
                  << ", old-loc=" << old_loc << ":" << coord_t(old_loc)
                  << ", new-loc=" << new_loc << ":" << coord_t(new_loc) << "):"
                  << std::endl;
#endif

        // this is the first time that we change loc in this value.
        // Compute new value and cache it for later use
        std::map<dai::Var, size_t> new_state = dai::cache_with_location_t::state(loc, value);
        assert(new_state.find(dai::cache_with_location_t::variable(num_locs_)) != new_state.end());
        assert(new_state.at(dai::cache_with_location_t::variable(num_locs_)) == size_t(old_loc));
        new_state.at(dai::cache_with_location_t::variable(num_locs_)) = new_loc;
        int new_value = dai::calcLinearState(varset, new_state);

        // cache new value and return
        assert((new_value >= 0) && (new_value < int(varset.nrStates())));
        patched_values_[loc][value] = new_value;
        return new_value;
    }
};

}; // namespace SLAM

#undef DEBUG

#endif

