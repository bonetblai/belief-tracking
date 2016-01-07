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

#ifndef EDBP_H
#define EDBP_H

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>

#include <dai/alldai.h>

//#define DEBUG

namespace Inference {

class edbp_t {
    // conversion between libdai and edbp factors
    static std::vector<std::vector<int> > edbp_factor_indices_;

    static void compute_edbp_factor_indices() {
        edbp_factor_indices_ = std::vector<std::vector<int> >(10);

        // define indices to compute
        std::vector<int> number_vars;
        number_vars.push_back(2);
        number_vars.push_back(3);
        number_vars.push_back(4);
        number_vars.push_back(5);
        number_vars.push_back(6);
        number_vars.push_back(9);

        for( int i = 0; i < int(number_vars.size()); ++i ) {
            int nvars = number_vars[i];
            edbp_factor_indices_[nvars] = std::vector<int>(1 << nvars);

            dai::VarSet vars, r_vars;
            for( int var = 0; var < nvars; ++var ) {
                vars |= dai::Var(var, 2);
                r_vars |= dai::Var(nvars - var - 1, 2);
            }
#ifdef DEBUG
            std::cout << "vars=" << vars << ", r_vars=" << r_vars << std::endl;
#endif

            for( int value = 0; value < (1 << nvars); ++value ) {
                std::map<dai::Var, size_t> state = dai::calcState(vars, value);
                std::map<dai::Var, size_t> r_state;
                for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it ) {
                    int label = nvars - 1 - it->first.label();
                    r_state[dai::Var(label, 2)] = it->second;
                }
                int r_index = dai::calcLinearState(r_vars, r_state);
                edbp_factor_indices_[nvars][r_index] = value;
#ifdef DEBUG
                std::cout << "State of vars(index=" << value << "): " //<< state
                          << "; state of r_vars(index=" << r_index << "): " //<< dai::calcState(r_vars, r_index)
                          << std::endl;
#endif
            }
        }
    }

  public:
    static void initialize() {
        compute_edbp_factor_indices();
    }
    static int edbp_factor_index(int nvars, int index) {
        assert(nvars < int(edbp_factor_indices_.size()));
        assert(index < int(edbp_factor_indices_[nvars].size()));
        return edbp_factor_indices_[nvars][index];
    }

    static int edbp_factor_index(const dai::Factor &factor, int index) {
        return edbp_factor_index(factor.vars().size(), index);
    }
};

}; // namespace Inference

#undef DEBUG

#endif

