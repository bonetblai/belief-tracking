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
#include <set>
#include <vector>

#include "arc_consistency.h"

#define DEBUG

namespace CSP {

template<typename T> class weighted_arc_consistency_t {
  public:
    typedef constraint_digraph_t::edge_list_t edge_list_t;
    typedef constraint_digraph_t::edge_t edge_t;

  private:
    edge_list_t worklist_;

  protected:
    int nvars_;
    std::vector<T*> domain_;
    const constraint_digraph_t &digraph_;

  public:
    weighted_arc_consistency_t(const constraint_digraph_t &digraph)
      : nvars_(digraph.nvars()), domain_(nvars_, static_cast<T*>(0)), digraph_(digraph) { }
    explicit weighted_arc_consistency_t(const weighted_arc_consistency_t &ac)
      : nvars_(ac.nvars_), domain_(ac.domain_), digraph_(ac.digraph_) { }
    weighted_arc_consistency_t(weighted_arc_consistency_t &&ac) = default;
    ~weighted_arc_consistency_t() { clear(); }

    int nvars() const { return nvars_; }
    const constraint_digraph_t& digraph() const { return digraph_; }
    bool is_consistent(int var) const { return !domain_[var]->empty(); }
    bool is_consistent() const {
        for( int var = 0; var < nvars_; ++var ) {
            if( !is_consistent(var) ) return false;
        }
        return true;
    }

    T* domain(int var) { return domain_[var]; }
    const T* domain(int var) const { return domain_[var]; }
    void set_domain(int var, T *domain) { domain_[var] = domain; }

    void clear() {
        for( int var = 0; var < nvars_; ++var )
            domain_[var] = 0;
    }
    void delete_domains_and_clear() {
        for( int var = 0; var < nvars_; ++var ) {
            delete domain_[var];
            domain_[var] = 0;
        }
    }
    void clear_domains() {
        for( int var = 0; var < nvars_; ++var ) {
            domain_[var]->clear();
        }
    }

    // used to insert edges into worklist prior to calling weighted-ac3
    void add_to_worklist(edge_t edge) {
        worklist_.push_back(edge);
    }
    void add_to_worklist(int seed_var) {
        worklist_.reserve(digraph_.nedges());
        if( seed_var > -1 ) {
            // add all edges pointing to seed var
            worklist_.insert(worklist_.end(),
                             digraph_.edges_pointing_to(seed_var).begin(),
                             digraph_.edges_pointing_to(seed_var).end());
        } else {
            // add every edge to worklist
            for( int node = 0; node < digraph_.nvars(); ++node ) {
                worklist_.insert(worklist_.end(),
                                 digraph_.edges_pointing_to(node).begin(),
                                 digraph_.edges_pointing_to(node).end());
            }
        }
    }
    void add_all_edges_to_worklist() {
        add_to_worklist(-1);
    }

    // check whether the partial assignment [ var_x = val_x, var_y = val_y ] is
    // consistent with the constraints. The preprocessing and postprocessing 
    // methods are called before and after making the consistency checks
    // associated with arc var_x -> var_y. By using them, one can save time in
    // some cases; but they are not strictly needed.
    //
    // Pre/post-processing is typically used to set auxiliary variables that
    // help in calculation of consistent(...).
    virtual void weighted_arc_reduce_preprocessing_0(int var_x, int var_y) const { }
    virtual void weighted_arc_reduce_preprocessing_1(int var_x, int val_x) const {
        std::cout << "error: must implement 'weighted_arc_reduce_preprocessing()' for arc consistency" << std::endl;
        assert(0);
    }
    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const { return false; }
    virtual void weighted_arc_reduce_postprocessing(int var_x, int var_y) const { }

    // reduce arc var_x -> var_y. That is, for each value of var_x, find a
    // compatible value for var_y. If such value is not found, the value of
    // var_x is removed from the domain of var_x. The method returns whether
    // some element of var_x is removed or not.
    bool weighted_arc_reduce(int var_x, int var_y) {
        assert(var_x != var_y);

        bool weight_change = false;
        weighted_arc_reduce_preprocessing_0(var_x, var_y);
        for( typename T::const_iterator it = domain_[var_x]->begin(); it != domain_[var_x]->end(); ++it ) {
            weighted_arc_reduce_preprocessing_1(var_x, *it);
            int min_weight = std::numeric_limits<int>::max();
            for( typename T::const_iterator jt = domain_[var_y]->begin(); jt != domain_[var_y]->end(); ++jt ) {
                if( consistent(var_x, var_y, *it, *jt) ) {
                    min_weight = std::min(min_weight, jt.weight());
                }
            }

            // should we remove valuations with max() value?
            // set weight of val_x to max of current weight and min_weight
            if( min_weight > it.weight() ) {
#ifdef DEBUG
                std::cout << "weighted-ac3: increasing weight of " << *it
                          << " in domain of var_x=" << var_x
                          << " to " << min_weight
                          << " [old_weight=" << it.weight()
                          << ", var_y=" << var_y << "]"
                          << std::endl;
#endif
                it.weight() = min_weight;
                weight_change = true;
            }
        }
        return weight_change;
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

