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


#ifndef ITERATED_WEIGHTED_ARC_CONSISTENCY_H
#define ITERATED_WEIGHTED_ARC_CONSISTENCY_H

#include <cassert>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

#include "arc_consistency.h"

//#define DEBUG

namespace CSP {

template<typename T> class iterated_weighted_arc_consistency_t : public arc_consistency_t<T> {
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
    using arc_consistency_t<T>::consistent;
    using arc_consistency_t<T>::add_to_worklist;
    using arc_consistency_t<T>::arc_reduce_preprocessing_0;
    using arc_consistency_t<T>::arc_reduce_preprocessing_1;
    using arc_consistency_t<T>::arc_reduce_postprocessing;

  public:
    iterated_weighted_arc_consistency_t(const constraint_digraph_t &digraph)
      : arc_consistency_t<T>(digraph) {
    }
    explicit iterated_weighted_arc_consistency_t(const iterated_weighted_arc_consistency_t &ac)
      : arc_consistency_t<T>(ac) {
    }
    iterated_weighted_arc_consistency_t(iterated_weighted_arc_consistency_t &&ac) = default;
    ~iterated_weighted_arc_consistency_t() { clear(); }

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

    // run standard arc-consistency in pruned domains with level i.
    // A pruned domain with level i consists of all valuations with
    // weight <= i. If there is no compatible value for valuation x,
    // increase its weight by 1 unit
    bool weighted_arc_reduce(int level, int var_x, int var_y) {
        assert(var_x != var_y);
        bool change = false;
        arc_reduce_preprocessing_0(var_x, var_y);
        for( typename T::const_iterator it = domain_[var_x]->begin(); it != domain_[var_x]->end(); ++it ) {
            if( it.weight() > level ) continue;
            arc_reduce_preprocessing_1(var_x, *it);
            bool found_compatible_value = false;
            for( typename T::const_iterator jt = domain_[var_y]->begin(); jt != domain_[var_y]->end(); ++jt ) {
                if( jt.weight() > level ) continue;
                if( consistent(var_x, var_y, *it, *jt) )
                    found_compatible = true;
            }

            // set weight of val_x to max of current weight and min_weight
            if( !found_compatible_value ) {
#ifdef DEBUG
                    std::cout << "weighted-ac3: increasing weight of valuation " << *it
                              << " in domain of var_x=" << var_x
                              << " to " << it.weight() + 1
                              << " [var_y=" << var_y << "]"
                              << std::endl;
#endif
                domain_[var_x]->set_weight(it.index(), 1 + it.weight());
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

    // iterated weighed AC3: iterated weighted arc consistency with given level
    // Returns true iff some weight changed
    bool weighted_ac3(int level, std::vector<int> &revised_vars, bool propagate = true) {

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
            if( weighted_arc_reduce(level, var_x, var_y) ) {
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

    bool iterated_weighted_ac3(int max_level, std::vector<int> &revised_vars, bool propagate = true) {
        std::vector<bool> set_revised_vars(nvars_, false);
        bool change = false;
        for( int level = 0; level < level; ++level ) {
            change = weighted_ac3(level, revised_vars, propagate) || change;
            for( int k = 0; k < int(revised_vars.size()); ++k )
                set_revised_vars[k] = true;
        }
        revised_vars.clear();
        for( int k = 0; k < int(set_revised_vars.size()); ++k ) {
            if( set_revised_vars[k] )
                revised_vars.push_back(k);
        }
        return change;
    }
};

}; // end of namespace CSP

#undef DEBUG

#endif

