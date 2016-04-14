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

#ifndef LIBDAI_CACHE_H
#define LIBDAI_CACHE_H

#include <cassert>
#include <iostream>
#include <map>
#include <vector>

#include <dai/alldai.h>

//#define DEBUG

namespace dai {

class cache_t {
  protected:
    static int num_locs_;
    static std::vector<dai::Var> variables_;
    static std::vector<dai::VarSet> varsets_;
    static std::vector<std::vector<std::map<dai::Var, size_t>*> > state_cache_;

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
            std::cout << "# cache_t: compute_basic_elements: loc=" << loc << ", vars=" << varsets_[loc] << std::endl;
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
        std::cout << "# cache_t: cache-for-states:"
                  << " size=" << size
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
        clean_cache_for_states();
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

    static void print_state(std::ostream &os, const std::map<dai::Var, size_t> &state) {
        os << "{";
        for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it )
            os << it->first << "=" << it->second << ",";
        os << "}";
    }
};

class cache_with_location_t {
  protected:
    static int num_locs_;
    static std::vector<dai::Var> variables_;
    static std::vector<dai::VarSet> varsets_;
    static std::vector<std::vector<std::map<dai::Var, size_t>*> > state_cache_;

    static void compute_basic_elements(int nrows, int ncols) {
        // variables for each location
        variables_ = std::vector<dai::Var>(1 + num_locs_);
        for( int loc = 0; loc < num_locs_; ++loc )
            variables_[loc] = dai::Var(loc, 2);
        variables_.back() = dai::Var(num_locs_, num_locs_);

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
            vars.push_back(variables_.back());
            std::sort(vars.begin(), vars.end());
            varsets_[loc] = dai::VarSet(vars.begin(), vars.end());
#ifdef DEBUG
            std::cout << "# cache_with_location_t: compute_basic_elements: loc=" << loc << ", vars=" << varsets_[loc] << std::endl;
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
        std::cout << "# cache_with_location_t: cache-for-states:"
                  << " size=" << size
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
    cache_with_location_t() { }
    ~cache_with_location_t() { }

    static void finalize() {
        clean_cache_for_states();
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

    static void print_state(std::ostream &os, const std::map<dai::Var, size_t> &state) {
        os << "{";
        for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it )
            os << it->first << "=" << it->second << ",";
        os << "}";
    }
};

}; // namespace dai

#undef DEBUG

#endif

