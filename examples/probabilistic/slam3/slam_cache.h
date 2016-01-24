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
            slabels_ = std::vector<std::vector<int> >(num_locs_, std::vector<int>(512, -1));
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
        const std::map<dai::Var, size_t> &state = cache_t::state(loc, value);
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

#if 0 // REMOVE
class cache_t {
  protected:
    static int num_locs_;
    static std::vector<dai::Var> variables_;
    static std::vector<dai::VarSet> varsets_;
    static std::vector<std::vector<std::map<dai::Var, size_t>*> > state_cache_;
    static std::vector<std::vector<int> > slabels_;

    static void compute_basic_elements(int nrows, int ncols) {
        // variables for each location
        variables_ = std::vector<dai::Var>(num_locs_);
        for( int loc = 0; loc < num_locs_; ++loc )
            variables_[loc] = dai::Var(loc, 2);

        // variable sets for each location
        varsets_ = std::vector<dai::VarSet>(num_locs_);
        for( int loc = 0; loc < num_locs_; ++loc ) {
            int row = loc / ncols, col = loc % ncols;
            std::vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= nrows) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= ncols) ) continue;
                    vars.push_back(variables_[nr * ncols + nc]);
                }
            }
            std::sort(vars.begin(), vars.end());
            varsets_[loc] = dai::VarSet(vars.begin(), vars.end());
#ifdef DEBUG
            std::cout << "# compute_basic_elements: loc=" << loc << ", vars=" << varsets_[loc] << std::endl;
#endif
        }
    }

    static void compute_cache_for_states(int nrows, int ncols) {
        float start_time = Utils::read_time_in_seconds();
        int size = 0;
        state_cache_ = std::vector<std::vector<std::map<dai::Var, size_t>*> >(num_locs_);
        for( int loc = 0; loc < num_locs_; ++loc ) {
            const dai::VarSet &varset = varsets_[loc];
            state_cache_[loc] = std::vector<std::map<dai::Var, size_t>*>(varset.nrStates(), static_cast<std::map<dai::Var, size_t>*>(0));
            for( int value = 0; value < int(varset.nrStates()); ++value ) {
                std::map<dai::Var, size_t> *state = new std::map<dai::Var, size_t>(dai::calcState(varset, value));
                state_cache_[loc][value] = state;
                ++size;
            }
        }
        std::cout << "# cache-for-states:"
                  << " size=" << size
                  //<< ", hits=" << cache_hits
                  << ", time=" << Utils::read_time_in_seconds() - start_time
                  << std::endl;
    }

    static void clean_cache_for_states() {
        for( int i = 0; i < int(state_cache_.size()); ++i ) {
            for( int j = 0; j < int(state_cache_[i].size()); ++j ) {
                delete state_cache_[i][j];
            }
        }
    }

  public:
    cache_t() { }
    ~cache_t() { }

    static void finalize() {
        for( int i = 0; i < int(state_cache_.size()); ++i ) {
            for( int j = 0; j < int(state_cache_[i].size()); ++j ) {
                std::map<dai::Var, size_t> *state = state_cache_[i][j];
                delete state;
            }
        }
    }

    static const std::vector<dai::Var>& variables() {
        return variables_;
    }
    static const dai::Var& variable(int loc) {
        return variables_[loc];
    }
    static const std::vector<dai::VarSet>& varsets() {
        return varsets_;
    }
    static const dai::VarSet& varset(int loc) {
        return varsets_[loc];
    }
    static const std::map<dai::Var, size_t>& state(int var, int value) {
        return *state_cache_[var][value];
    }

    static int get_slabels(int loc, const dai::VarSet &varset, int value, int var_offset(int, int)) {
        assert((loc >= 0) && (loc < num_locs_));
        assert((value >= 0) && (value < int(varset.nrStates())));

        // allocate cache if this is first call
        if( slabels_.empty() )
            slabels_ = std::vector<std::vector<int> >(num_locs_, std::vector<int>(512, -1));
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
        const std::map<dai::Var, size_t> &state = cache_t::state(loc, value);
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

    static void print_state(std::ostream &os, const std::map<dai::Var, size_t> &state) {
        os << "{";
        for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it )
            os << it->first << "=" << it->second << ",";
        os << "}";
    }
};
#endif

}; // namespace SLAM

#undef DEBUG

#endif

