/*
 *  Copyright (C) Universidad Simon Bolivar
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

#ifndef DECOMPOSABLE_H
#define DECOMPOSABLE_H

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

#include "cellmap.h"
#include "slam_cache.h"
#include "inference.h"
#include "edbp.h"
#include "utils.h"

//#define DEBUG

namespace Decomposable {

class cache_t : public SLAM::cache_with_location_t {
  public:
    cache_t() { }
    ~cache_t() { }

    static void initialize(int nrows, int ncols) {
        num_locs_ = nrows * ncols;
        SLAM::cache_with_location_t::compute_basic_elements(nrows, ncols);
        SLAM::cache_with_location_t::compute_cache_for_states(nrows, ncols);
    }
    static void finalize() {
        SLAM::cache_with_location_t::finalize();
    }
};

template <typename BASE> class decomposable_t : public tracking_t<BASE> {
    using tracking_t<BASE>::base_;

  protected:
    // factors
    std::vector<dai::Factor> factors_;

    // marginals over single variables
    std::vector<dai::Factor> marginals_on_vars_;

    // store inv. transition model for update()
    std::vector<std::pair<int, float> > *sparse_inv_tr_vec_;
    std::vector<std::pair<int, float> > sparse_inv_tr_;

    // computation of marginals in factor model
    mutable std::vector<int> indices_for_updated_factors_;
    mutable std::vector<dai::Factor> marginals_;
    Inference::inference_t inference_;

  public:
    decomposable_t(const std::string &name, const BASE &base, const Inference::inference_t *i)
      : tracking_t<BASE>(name, base) {
        factors_ = std::vector<dai::Factor>(base_.nloc_);
        marginals_ = std::vector<dai::Factor>(base_.nloc_);

        for( int loc = 0; loc < base_.nloc_; ++loc ) {
            factors_[loc] = dai::Factor(cache_t::varset(loc));
            marginals_[loc] = dai::Factor(cache_t::varset(loc));
        }

        if( i != 0 ) inference_ = *i;

        // create storage for inv. transition model
        sparse_inv_tr_vec_ = new std::vector<std::pair<int, float> >[base_.nloc_];
        sparse_inv_tr_.reserve(base_.nloc_);
    }
    decomposable_t(const std::string &name, const BASE &base, const std::multimap<std::string, std::string> &parameters)
      : decomposable_t(name, base, 0) {

        std::multimap<std::string, std::string>::const_iterator it = parameters.find("inference");
        if( it != parameters.end() ) {
            inference_.set_inference_algorithm(it->second, "BEL", false);
            inference_.create_and_initialize_algorithm(factors_);
        }
    }
    virtual ~decomposable_t() {
        delete[] sparse_inv_tr_vec_;
    }

    // CHECK: don't remember this stuff below. Must move this into cellmap_t as slam-type should be hidden
    static const BASE *static_base_;
    static int var_offset_corridor_slam(int loc, int var_id) {
        return 1 + var_id - loc;
    }
    static int var_offset_mine_mapping(int loc, int var_id) {
        assert(static_base_ != 0);
        return static_base_->var_offset(loc, var_id);
    }
    int get_slabels(int loc, const dai::VarSet &varset, int value) const {
        int (*var_offset)(int loc, int var_id) = 0;
        if( base_.slam_type_ == cellmap_t::CORRIDOR ) {
            var_offset = var_offset_corridor_slam;
        } else if( (base_.slam_type_ == cellmap_t::MINE_MAPPING_PEAKED) || (base_.slam_type_ == cellmap_t::MINE_MAPPING_NON_PEAKED) ) {
            var_offset = var_offset_mine_mapping;
        }
        assert(var_offset != 0);
        return cache_t::get_slabels(loc, varset, value, var_offset);
    }

    void inference(bool print_marginals = false) const {
        //std::cout << "inference(): entry" << std::endl;
        inference_.calculate_marginals(cache_t::variables(),
                                       indices_for_updated_factors_,
                                       factors_,
                                       marginals_,
                                       Inference::edbp_t::edbp_factor_index,
                                       print_marginals);
        assert(indices_for_updated_factors_.empty());
        //std::cout << "inference(): exit" << std::endl;
    }

    virtual void clear() {
    }

    virtual void initialize() {
        //std::cout << "initialize(): entry" << std::endl;
        for( int loc = 0; loc < base_.nloc_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / float(factor.nrStates() / base_.nloc_);
            for( int value = 0; value < int(factor.nrStates()); ++value ) {
                const std::map<dai::Var, size_t> &state = cache_t::state(loc, value);
                assert(state.find(cache_t::variable(base_.nloc_)) != state.end());
                if( base_.initial_loc_ == int(state.at(cache_t::variable(base_.nloc_))) )
                    factor.set(value, p);
                else
                    factor.set(value, 0);
            }
            indices_for_updated_factors_.push_back(loc);
        }
        inference();
        calculate_marginals();
        //std::cout << "initialize(): exit" << std::endl;
    }

    virtual void update(int last_action, int obs) {
        //std::cout << "update: entry: last-action=" << last_action << ", obs=" << obs << std::endl;

        // calculate P(loc(t)|loc(t+1),m(t+1),a(t),h(t)):
        //   P(loc(t)|loc(t+1),m(t+1),a(t),h(t)) = alpha * P(loc(t+1)|loc(t),m(t+1),a(t),h(t)) * P(loc(t)|m(t+1),a(t),h(t))
        //                                       = alpha * P(loc(t+1)|loc(t),a(t)) * P(loc(t)|h(t))
        //                                       = alpha * tr(loc(t+1)|loc(t),a(t)) * P(loc(t)|h(t)) [tr. dyn. * joint marginal]
        assert(int(marginals_on_vars_.size()) == 1 + base_.nloc_);
        assert(int(marginals_on_vars_[base_.nloc_].nrStates()) == base_.nloc_);
        dai::Factor &loc_marginal = marginals_on_vars_[base_.nloc_];
        assert(base_.nloc_ == int(loc_marginal.nrStates()));
        for( int new_loc = 0; new_loc < base_.nloc_; ++new_loc ) {
            sparse_inv_tr_.clear();
            float alpha = 0;
            for( int loc = 0; loc < base_.nloc_; ++loc ) {
                float tr = base_.probability_tr_loc(last_action, loc, new_loc);
                assert(tr * loc_marginal[loc] >= 0);
                if( tr * loc_marginal[loc] != 0 )
                    sparse_inv_tr_.push_back(std::make_pair(loc, tr * loc_marginal[loc]));
                alpha += tr * loc_marginal[loc];
            }
            if( alpha != 0 ) {
                for( int loc = 0; loc < int(sparse_inv_tr_.size()); ++loc )
                    sparse_inv_tr_[loc].second /= alpha;
                sparse_inv_tr_vec_[new_loc] = sparse_inv_tr_;
            }
        }

        // calculate new factors replacing regression on loc by marginalization
        for( int j = 0; j < base_.nloc_; ++j ) {
            dai::Factor &factor = factors_[j];
            dai::Factor new_factor(cache_t::varset(j));
            bool change = false;
            for( int value = 0; value < int(factor.nrStates()); ++value ) {
                const std::map<dai::Var, size_t> &state = cache_t::state(j, value);
                assert(state.find(cache_t::variable(base_.nloc_)) != state.end());
                int new_loc = state.at(cache_t::variable(base_.nloc_));

                float p = 0;
                const std::vector<std::pair<int, float> > &inv_tr = sparse_inv_tr_vec_[new_loc];
                for( int i = 0; i < int(inv_tr.size()); ++i ) {
                    int loc = inv_tr[i].first;
                    assert(inv_tr[i].second != 0);
                    float tr = base_.probability_tr_loc(last_action, loc, new_loc);
                    int value_with_replaced_loc = cache_t::replace_loc_in_value(j, factor.vars(), value, new_loc, loc);
                    p += inv_tr[i].second * tr * factor[value_with_replaced_loc];
                }

                if( new_loc == j ) {
                    int slabels = get_slabels(j, factor.vars(), value);
                    new_factor.set(value, p * base_.probability_obs(obs, j, slabels, last_action));
                } else {
                    assert(base_.nlabels_ == 2);
                    if( base_.slam_type_ == cellmap_t::MINE_MAPPING_NON_PEAKED )
                        new_factor.set(value, p / 10.0);
                    else
                        new_factor.set(value, p / float(base_.nlabels_));
                }
                change = change || (factor[value] != new_factor[value]);
            }

            try {
                if( change ) {
                    new_factor.normalize();
                    factor = new_factor;
                    indices_for_updated_factors_.push_back(j);
                }
            } catch( dai::Exception &e ) {
                std::cout << "error: dai exception: " << e.getMsg() << std::endl;
                assert(0);
            }
        }

        inference();
        //calculate_marginals();
        //std::cout << "update: exit" << std::endl;
    }

    virtual void calculate_marginals() {
        //std::cout << "calculate_marginals(): entry" << std::endl;
        // initialize marginals
        marginals_on_vars_ = std::vector<dai::Factor>(base_.nvars_);
        for( int var = 0; var < base_.nvars_; ++var )
            marginals_on_vars_[var] = dai::Factor(dai::VarSet(dai::Var(var, base_.variable_size(var))), 0.0);

        // marginalize beams to get marginals over variables
        for( int loc = 0; loc < base_.nloc_; ++loc ) {
            dai::Factor marginal = marginals_[loc].marginal(cache_t::variable(loc));
            assert(base_.nlabels_ == int(marginal.nrStates()));
            for( int label = 0; label < base_.nlabels_; ++label )
                marginals_on_vars_[loc].set(label, marginal[label]);
        }
        dai::Factor marginal = marginals_[0].marginal(cache_t::variable(base_.nloc_));
        assert(base_.nloc_ == int(marginal.nrStates()));
        for( int loc = 0; loc < base_.nloc_; ++loc )
                marginals_on_vars_[base_.nloc_].set(loc, marginal[loc]);

        // verify
        for( int var = 0; var < base_.nvars_; ++var ) {
            float sum = 0;
            for( int value = 0; value < base_.variable_size(var); ++value )
                sum += marginals_on_vars_[var][value];
            if( fabs(sum - 1.0) >= EPSILON ) {
                std::cout << "warning: decomposable::calculate_marginals(): sum=" << sum << std::endl;
                //assert(fabs(sum - 1.0) < EPSILON);
            }
        }
        //std::cout << "calculate_marginals(): exit" << std::endl;
    }
    virtual void get_marginal(int var, dai::Factor &marginal) const {
        marginal = marginals_on_vars_[var];
    }
    virtual float* get_marginal(int var, float *ptr) const {
        const dai::Factor &factor = marginals_on_vars_[var];
        for( int value = 0; value < int(factor.nrStates()); ++value )
            *ptr++ = factor[value];
        return ptr;
    }

    virtual std::string id() const {
        std::string id_str;
        id_str = std::string("decomposable(") +
          std::string("inference=") + inference_.id() +
          std::string(")");
        return id_str;
    }

    virtual void print(std::ostream &os) const {
        assert(0); // CHECK
    }
};

}; // namespace Decomposable

#undef DEBUG

#endif

