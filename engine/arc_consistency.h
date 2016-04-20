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


#ifndef ARC_CONSISTENCY_H
#define ARC_CONSISTENCY_H

#include <cassert>
#include <iostream>
#include <set>
#include <vector>

//#define DEBUG

namespace CSP {

class constraint_digraph_t {
  public:
    typedef std::pair<int, int> edge_t;
    typedef std::vector<int> node_list_t;
    typedef std::vector<edge_t> edge_list_t;

  protected:
    int nvars_;
    int nedges_;
    std::vector<edge_list_t> edges_pointing_to_;

  public:
    constraint_digraph_t(int nvars = 0) {
        create_empty_graph(nvars);
    }
    constraint_digraph_t(const constraint_digraph_t &digraph)
      : nvars_(digraph.nvars_), nedges_(digraph.nedges_),
        edges_pointing_to_(digraph.edges_pointing_to_) {
    }
    constraint_digraph_t(constraint_digraph_t &&digraph) = default;
    ~constraint_digraph_t() { }

    void create_empty_graph(int nvars) {
        nvars_ = nvars;
        nedges_ = 0;
        edges_pointing_to_ = std::vector<edge_list_t>(nvars_);
    }

    void clear() {
        nedges_ = 0;
        edges_pointing_to_ = std::vector<edge_list_t>(nvars_);
    }

    int nvars() const { return nvars_; }
    int nedges() const { return nedges_; }

    const edge_list_t& edges_pointing_to(int var) const {
        return edges_pointing_to_[var];
    }

    void reserve_edge_list(int var, int reservation) {
        edges_pointing_to_[var].reserve(reservation);
    }

    void add_edge(int var_x, int var_y) {
        ++nedges_;
        edges_pointing_to_[var_y].push_back(std::make_pair(var_x, var_y));
    }
};

template<typename T> class arc_consistency_t {
  public:
    typedef constraint_digraph_t::edge_list_t edge_list_t;
    typedef constraint_digraph_t::edge_t edge_t;

  protected:
    int nvars_;
    std::vector<T*> domain_;
    const constraint_digraph_t &digraph_;

    edge_list_t worklist_;

  public:
    arc_consistency_t(const constraint_digraph_t &digraph)
      : nvars_(digraph.nvars()), domain_(nvars_, static_cast<T*>(0)), digraph_(digraph) {
    }
    explicit arc_consistency_t(const arc_consistency_t &ac)
      : nvars_(ac.nvars_), domain_(ac.domain_), digraph_(ac.digraph_) {
    }
    //arc_consistency_t(arc_consistency_t &&ac) = default; // CHECK: when this is enabled, segfault occurs
    virtual ~arc_consistency_t() { clear(); }

    const arc_consistency_t<T>& operator=(const arc_consistency_t<T> &ac) {
        nvars_ = ac.nvars_;
        domain_ = ac.domain_;
        assert(&digraph_ == &ac.digraph_);
        worklist_ = ac.worklist_;
        return *this;
    }

    int nvars() const { return nvars_; }
    const constraint_digraph_t& digraph() const { return digraph_; }
    bool is_consistent() const {
        for( int var = 0; var < nvars_; ++var ) {
            if( !is_consistent(var) ) return false;
        }
        return true;
    }
    virtual bool is_consistent(int var) const {
        return !domain_[var]->empty();
    }

    T* domain(int var) { return domain_[var]; }
    const T* domain(int var) const { return domain_[var]; }
    void set_domain(int var, T *domain) { domain_[var] = domain; }

    void clear() {
        for( int var = 0; var < nvars_; ++var )
            domain_[var] = 0;
    }
    void delete_domains_and_clear() {
        for( int var = 0; var < nvars_; ++var )
            delete domain_[var];
        domain_.clear();
    }
    void clear_domains() {
        for( int var = 0; var < nvars_; ++var )
            domain_[var]->clear();
    }

    // used to insert edges into worklist prior to calling ac3.
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
    const edge_list_t& worklist() const {
        return worklist_;
    }

    // check whether the partial assignment [ var_x = val_x, var_y = val_y ] is
    // consistent with the constraints. The preprocessing and postprocessing 
    // methods are called before and after making the consistency checks
    // associated with arc var_x -> var_y. By using them, one can save time in
    // some cases; but they are not strictly needed.
    //
    // Pre/post-processing is typically used to set auxiliary variables that
    // help in calculation of consistent(...).
    virtual void arc_reduce_preprocessing_0(int var_x, int var_y) const { }
    virtual void arc_reduce_preprocessing_1(int var_x, int val_x) const {
        std::cout << "error: must implement 'arc_reduce_preprocessing()' for arc consistency" << std::endl;
        assert(0);
    }
    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const { return false; }
    virtual void arc_reduce_postprocessing(int var_x, int var_y) const { }

    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y) const { }
    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y, int val_y) const {
        std::cout << "error: must implement 'arc_reduce_inverse_check_preprocessing()' for inverse-check in arc consistency" << std::endl;
        assert(0);
    }
    virtual bool arc_reduce_inverse_check(int val_x) const { return false; }
    virtual void arc_reduce_inverse_check_postprocessing(int var_x, int var_y) const { }

    // reduce arc var_x -> var_y. That is, for each value of var_x, find a
    // compatible value for var_y. If such value is not found, the value of
    // var_x is removed from the domain of var_x. The method returns whether
    // some element of var_x is removed or not.
    bool arc_reduce(std::vector<bool> &revised_vars, int var_x, int var_y, bool inverse_check = false) {
        assert(var_x != var_y);
        std::vector<int> indices_to_erase;
        indices_to_erase.reserve(domain_[var_x]->size());

        if( !inverse_check ) {
            arc_reduce_preprocessing_0(var_x, var_y);
            for( typename T::const_iterator it = domain_[var_x]->begin(); it != domain_[var_x]->end(); ++it ) {
                arc_reduce_preprocessing_1(var_x, *it);
                bool found_compatible_value = false;
                for( typename T::const_iterator jt = domain_[var_y]->begin(); jt != domain_[var_y]->end(); ++jt ) {
                    if( consistent(var_x, var_y, *it, *jt) ) {
                        found_compatible_value = true;
                        break;
                    }
                }
                if( !found_compatible_value ) {
#ifdef DEBUG
                    std::cout << "ac3: removing " << *it
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
        } else {
            arc_reduce_inverse_check_preprocessing(var_x, var_y);
            for( typename T::const_iterator it = domain_[var_y]->begin(); it != domain_[var_y]->end(); ++it )
                arc_reduce_inverse_check_preprocessing(var_x, var_y, *it);

            for( typename T::const_iterator it = domain_[var_x]->begin(); it != domain_[var_x]->end(); ++it ) {
                if( !arc_reduce_inverse_check(*it) ) {
#ifdef DEBUG
                    std::cout << "ac3: removing " << *it
                              << " from domain of var_x=" << var_x
                              << " [var_y=" << var_y << "]"
                              << std::endl;
#endif
                    indices_to_erase.push_back(it.index());
                    revised_vars[var_x] = true;
                }
            }
            domain_[var_x]->erase_ordered_indices(indices_to_erase);
            arc_reduce_inverse_check_postprocessing(var_x, var_y);
        }
        return !indices_to_erase.empty();
    }

    // revise arc var_x -> var_y. This method is called after the reduction of
    // the edge removed some element in the domain of var_x. Then, all edges
    // ?var -> var_x such that ?var is different from var_y must be scheduled
    // for revision.
    void arc_revise(int var_x, int var_y) {
        const edge_list_t &edges = digraph_.edges_pointing_to(var_x);
        for( int i = 0, isz = edges.size(); i < isz; ++i ) {
            int nvar = edges[i].first;
            assert(nvar != var_x);
            if( nvar != var_y ) {
                add_to_worklist(std::make_pair(nvar, var_x));
            }
        }
    }

    // AC3: Arc Consistency. Time is O(ed^3) where e is #edges in constraint
    // graph and d is size of larges domain: there are at most 2ed arc revisions
    // since each revision or arc (y,x) is caused by a removed element from the
    // domain of x (D_x), and each such revision takes time d^2 because for each
    // element of D_x, a consistent element of D_y must be found. Space is O(e)
    // since this is the maximum size of the worklist.
    //
    // Returns true iff some value is removed from some domain
    bool ac3(std::vector<bool> &revised_vars, bool propagate = true, bool inverse_check = false) {
        assert(int(revised_vars.size()) >= nvars_);

        // revise arcs until worklist becomes empty
        bool something_removed = false;
        while( !worklist_.empty() ) {
            edge_t edge = worklist_.back();
            worklist_.pop_back();
            int var_x = edge.first;
            int var_y = edge.second;

#ifdef DEBUG
            std::cout << "ac3: revise arc " << var_x << " --> " << var_y << std::endl;
#endif

            // try to reduce arc var_x -> var_y
            if( arc_reduce(revised_vars, var_x, var_y, inverse_check) ) {
                something_removed = true;
                if( domain_[var_x]->empty() ) {
                    // domain of var_x became empty. This means that the
                    // CSP is inconsistent. Clear all other domains.
                    clear_domains();
                    worklist_.clear();
                    return true;
                } else {
                    // some element was removed from domain of var_x,
                    // schedule revision of all arcs pointing to var_x
                    // different from arc the arc var_y -> var_x
                    if( propagate ) {
                        arc_revise(var_x, var_y);
                    } else {
                        assert(0); // delayed propagation (not yet implemented)
                    }
                }
            }
        }
        return something_removed;
    }
    bool ac3(std::vector<int> &revised_vars, bool propagate = true, bool inverse_check = false) {
        std::vector<bool> revised_vars_tmp(nvars_, false);
        bool something_removed = ac3(revised_vars_tmp, propagate, inverse_check);
        revised_vars.clear();
        for( int var = 0; var < nvars_; ++var ) {
            if( revised_vars_tmp[var] )
                revised_vars.push_back(var);
        }
        return something_removed;
    }
};

}; // end of namespace CSP

#undef DEBUG

#endif

