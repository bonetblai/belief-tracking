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
    mutable int max_allowed_weight_;
    mutable int cutoff_threshold_;

    using arc_consistency_t<T>::nvars_;
    using arc_consistency_t<T>::domain_;
    using arc_consistency_t<T>::digraph_;
    using arc_consistency_t<T>::worklist_;

  public:
    using arc_consistency_t<T>::clear;
    using arc_consistency_t<T>::consistent;
    using arc_consistency_t<T>::add_to_worklist;
    using arc_consistency_t<T>::arc_reduce_preprocessing_0;
    using arc_consistency_t<T>::arc_reduce_preprocessing_1;
    using arc_consistency_t<T>::arc_reduce_postprocessing;

  public:
    weighted_arc_consistency_t(const constraint_digraph_t &digraph)
      : arc_consistency_t<T>(digraph),
        max_allowed_weight_(std::numeric_limits<int>::max()),
        cutoff_threshold_(std::numeric_limits<int>::max()) {
    }
    explicit weighted_arc_consistency_t(const weighted_arc_consistency_t &ac)
      : arc_consistency_t<T>(ac),
        max_allowed_weight_(ac.max_allowed_weight_),
        cutoff_threshold_(ac.cutoff_threshold_) {
    }
    weighted_arc_consistency_t(weighted_arc_consistency_t &&ac) = default;
    ~weighted_arc_consistency_t() { clear(); }

    virtual bool is_consistent(int var) const {
        return domain_[var]->min_weight() != std::numeric_limits<int>::max();
    }

    int max_allowed_weight() const {
        return max_allowed_weight_;
    }
    int cutoff_threshold() const {
        return cutoff_threshold_;
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
    bool weighted_arc_reduce(int var_x, int var_y) {
        assert(var_x != var_y);
        bool change = false;
        arc_reduce_preprocessing_0(var_x, var_y);
        for( typename T::const_iterator it = domain_[var_x]->begin(); it != domain_[var_x]->end(); ++it ) {
            if( it.weight() >= cutoff_threshold_ ) continue;
            arc_reduce_preprocessing_1(var_x, *it);
            bool found_compatible_value = false;
            int min_weight = max_allowed_weight_;
            for( typename T::const_iterator jt = domain_[var_y]->begin(); jt != domain_[var_y]->end(); ++jt ) {
                if( (jt.weight() < cutoff_threshold_) && consistent(var_x, var_y, *it, *jt) ) {
                    min_weight = std::min(min_weight, jt.weight());
                    found_compatible_value = true;
                }
            }
            assert(min_weight <= max_allowed_weight_);

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
                assert(it.weight() == min_weight);
                change = true;
            } else if( !found_compatible_value ) {
#ifdef DEBUG
                std::cout << "weighted-ac3: increasing weight of valuation " << *it
                          << " in domain of var_x=" << var_x
                          << " to " << cutoff_threshold_
                          << " [old_weight=" << it.weight()
                          << ", var_y=" << var_y << "]"
                          << std::endl;
#endif
                domain_[var_x]->set_weight(it.index(), cutoff_threshold_);
                change = true;
            }
        }
        arc_reduce_postprocessing(var_x, var_y);
        return change;
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
    bool weighted_ac3(std::vector<int> &revised_vars, bool propagate = true) {

        // allocate space for revised vars
        std::vector<bool> inserted(nvars_, false);
        revised_vars.reserve(nvars_);
        revised_vars.clear();

        // revise arcs until worklist becomes empty
        bool weight_change = false;
        while( !worklist_.empty() ) {
            edge_t edge = worklist_.back();
            worklist_.pop_back();
            int var_x = edge.first;
            int var_y = edge.second;

            // keep record of revised vars
            if( !inserted[var_y] ) {
                inserted[var_y] = true;
                revised_vars.push_back(var_y);
            }

#ifdef DEBUG
            std::cout << "weighed-ac3: revise arc " << var_x << " --> " << var_y << std::endl;
#endif

            // try to reduce arc var_x -> var_y
            if( weighted_arc_reduce(var_x, var_y) ) {
                weight_change = true;

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
        return weight_change;
    }
};

}; // end of namespace CSP

#undef DEBUG

#endif

