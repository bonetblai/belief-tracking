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


#ifndef WEIGHTED_ARC_CONSISTENCY_H
#define WEIGHTED_ARC_CONSISTENCY_H

#include <cassert>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

#include "arc_consistency.h"

//#define DEBUG

namespace CSP {

template<typename T> class weighted_arc_consistency_t : public arc_consistency_t<T> {
  public:
    typedef constraint_digraph_t::edge_list_t edge_list_t;
    typedef constraint_digraph_t::edge_t edge_t;

  protected:
    using arc_consistency_t<T>::nvars_;
    using arc_consistency_t<T>::domain_;
    using arc_consistency_t<T>::digraph_;
    using arc_consistency_t<T>::worklist_;

  public:
    using arc_consistency_t<T>::clear;
    using arc_consistency_t<T>::clear_domains;
    using arc_consistency_t<T>::consistent;
    using arc_consistency_t<T>::add_to_worklist;
    using arc_consistency_t<T>::arc_reduce_preprocessing_0;
    using arc_consistency_t<T>::arc_reduce_preprocessing_1;
    using arc_consistency_t<T>::arc_reduce_postprocessing;

  public:
    weighted_arc_consistency_t(const constraint_digraph_t &digraph)
      : arc_consistency_t<T>(digraph) {
    }
    explicit weighted_arc_consistency_t(const weighted_arc_consistency_t &ac)
      : arc_consistency_t<T>(ac) {
    }
    weighted_arc_consistency_t(weighted_arc_consistency_t &&ac) = default;
    ~weighted_arc_consistency_t() {
        clear();
    }

    virtual bool is_consistent(int var) const {
        return domain_[var]->min_weight() != std::numeric_limits<int>::max();
    }

    int max_weight(int var) const {
        int weight = std::numeric_limits<int>::min();
        for( typename T::const_iterator it = domain_[var]->begin(); it != domain_[var]->end(); ++it )
            weight = std::max(weight, it.weight());
        return weight;
    }
    int max_weight() const {
        int weight = std::numeric_limits<int>::min();
        for( int var = 0; var < nvars_; ++var )
            weight = std::max(weight, max_weight(var));
        return weight;
    }

    // reduce arc var_x -> var_y. That is, for each value of var_x, find a
    // compatible value for var_y. If such value is not found, the value of
    // var_x is removed from the domain of var_x. The method returns whether
    // some element of var_x is removed or not.
    bool weighted_arc_reduce(std::vector<bool> &revised_vars, int var_x, int var_y) {
        assert(var_x != var_y);
        std::vector<int> indices_to_erase;
        indices_to_erase.reserve(domain_[var_x]->size());

        arc_reduce_preprocessing_0(var_x, var_y);
        for( typename T::const_iterator it = domain_[var_x]->begin(); it != domain_[var_x]->end(); ++it ) {
            arc_reduce_preprocessing_1(var_x, *it);
            bool found_compatible_value = false;
            int min_weight = std::numeric_limits<int>::max();
            for( typename T::const_iterator jt = domain_[var_y]->begin(); jt != domain_[var_y]->end(); ++jt ) {
                if( consistent(var_x, var_y, *it, *jt) ) {
                    min_weight = std::min(min_weight, jt.weight());
                    found_compatible_value = true;
                    if( min_weight == 0 ) break;
                }
            }

            // set weight of val_x to max of current weight and min_weight
            if( found_compatible_value && (min_weight > it.weight()) ) {
#ifdef DEBUG
                std::cout << "weighted-ac3: increasing weight of valuation " << *it
                          << " in domain of var_x=" << var_x
                          << " to " << min_weight
                          << " [old_weight=" << it.weight()
                          << ", var_y=" << var_y << "]"
                          << std::endl;
#endif
                domain_[var_x]->set_weight(it.index(), min_weight);
                revised_vars[var_x] = true;
            } else if( !found_compatible_value ) {
#ifdef DEBUG
                std::cout << "weighted-ac3: removing " << *it
                          << " from domain of var_x=" << var_x
                          << " [var_y=" << var_y << "]"
                          << std::endl;
#endif
                indices_to_erase.push_back(it.index());
                revised_vars[var_x] = true;
            }
        }
        domain_[var_x]->erase_ordered_indices(indices_to_erase);
        arc_reduce_postprocessing(var_x, var_y);
        return !indices_to_erase.empty();
    }

    // revise arc var_x -> var_y. This method is called after the reduction of
    // the edge removed some element in the domain of var_x. Then, all edges
    // ?var -> var_x such that ?var is different from var_y must be scheduled
    // for revision.
    void weighted_arc_revise(int var_x, int var_y) {
        const edge_list_t &edges = digraph_.edges_pointing_to(var_x);
        for( int i = 0, isz = edges.size(); i < isz; ++i ) {
            int nvar = edges[i].first;
            assert(nvar != var_x);
            if( nvar != var_y ) {
                add_to_worklist(std::make_pair(nvar, var_x));
            }
        }
    }

    // Weighed AC3: Weighted Arc Consistency.
    // Returns true iff some weight changed
    bool weighted_ac3(std::vector<bool> &revised_vars, bool propagate = true) {
        assert(int(revised_vars.size()) >= nvars_);

        // revise arcs until worklist becomes empty
        bool weight_change = false;
        while( !worklist_.empty() ) {
            edge_t edge = worklist_.back();
            worklist_.pop_back();
            int var_x = edge.first;
            int var_y = edge.second;

#ifdef DEBUG
            std::cout << "weighted-ac3: revise arc " << var_x << " --> " << var_y << std::endl;
#endif

            // try to reduce arc var_x -> var_y
            if( weighted_arc_reduce(revised_vars, var_x, var_y) ) {
                weight_change = true;
                if( domain_[var_x]->empty() ) {
                    // domain of var_x became empty. This means that the
                    // CSP is inconsistent. Clear all other domains.
                    clear_domains();
                    worklist_.clear();
                    return true;
                } else {
                    // some weight in var_x changed, schedule revision
                    // of all arcs pointing to var_x different from the
                    // arc var_y -> var_x
                    if( propagate ) {
                        weighted_arc_revise(var_x, var_y);
                    } else {
                        assert(0); // delayed propagation (not yet implemented)
                    }
                }
            }
        }
        return weight_change;
    }
    bool weighted_ac3(std::vector<int> &revised_vars, bool propagate = true) {
        std::vector<bool> revised_vars_tmp(nvars_, false);
        bool change = weighted_ac3(revised_vars_tmp, propagate);
        revised_vars.clear();
        for( int var = 0; var < nvars_; ++var ) {
            if( revised_vars_tmp[var] )
                revised_vars.push_back(var);
        }
        return change;
    }
};

}; // end of namespace CSP

#undef DEBUG

#endif

