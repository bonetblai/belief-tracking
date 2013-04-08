/*
 *  Copyright (C) 2012 Universidad Simon Bolivar
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

#ifndef WUMPUS_BASE_POLICY_H
#define WUMPUS_BASE_POLICY_H

#include "problem.h"

#include <problem.h>
#include <heuristic.h>

#include <cassert>
#include <iostream>
#include <queue>
#include <stdlib.h>
#include <limits.h>

//#define DEBUG


namespace Wumpus {

namespace Diagonal {

struct open_list_cmp {
    bool operator()(const std::pair<int, int> &p1, const std::pair<int, int> &p2) {
        return p1.second > p2.second;
    }
};

template<typename T> class wumpus_distances_t {
    int n_move_actions_;
    bool nesw_movements_;
    mutable std::vector<int> distances_;

    mutable std::priority_queue<std::pair<int, int>,
                                std::vector<std::pair<int, int> >,
                                open_list_cmp> queue_;

    void _initialize_distances() const {
        int dim = nesw_movements_ ? nrows_ * ncols_ : 4 * nrows_ * ncols_;
        distances_ = std::vector<int>(dim, INT_MAX);
    }

    void _setup_goal_pos(int p) const {
        distances_[p] = 0;
        queue_.push(std::make_pair(p, 0));
    }

    void _compute_distances(const T &state, bool safe) const {
        assert(nesw_movements_);
        assert(!nesw_movements_ || ((int)distances_.size() == nrows_ * ncols_));
        assert(nesw_movements_ || ((int)distances_.size() == 4 * nrows_ * ncols_));

        while( !queue_.empty() ) {
            std::pair<int, int> p = queue_.top();
            queue_.pop();
            assert(p.second < INT_MAX);
            if( p.second <= distances_[p.first] ) {
                int node = p.first;
                int cell = nesw_movements_ ? node : (node >> 2);
                int heading = nesw_movements_ ? -1 : (node & 0x3);
                for( int action = 0; action < n_move_actions_; ++action ) {
                    int ncell = cell, nheading = heading;
                    ncell = target_cell(cell, heading, action, nrows_, ncols_, nesw_movements_);
                    if( state.hazard_at(ncell) ) continue;
                    if( safe && !state.no_hazard_at(ncell) ) continue;

                    int cost = p.second == INT_MAX ? p.second : 1 + p.second;
                    int new_node = nesw_movements_ ? ncell : ((ncell << 2) + nheading);
                    if( cost < distances_[new_node] ) {
#ifdef DEBUG
                        std::cout << "new: a=" << action
                                  << ", node=(" << (ncell % ncols_)
                                  << "," << (ncell / ncols_)
                                  << "," << heading_name(nheading)
                                  << "), parent=(" << (cell % ncols_)
                                  << "," << (cell / ncols_)
                                  << "," << heading_name(heading)
                                  << "), cost=" << cost << std::endl;
#endif
                        distances_[new_node] = cost;
                        queue_.push(std::make_pair(new_node, cost));
                    }
                }
            }
        }
        //print(std::cout);
    }

  protected:
    int nrows_;
    int ncols_;

  public:
    wumpus_distances_t(int nrows, int ncols, bool nesw_movements)
      : n_move_actions_(0), nesw_movements_(nesw_movements), nrows_(nrows), ncols_(ncols) {
        assert(nesw_movements_);
        n_move_actions_ = nesw_movements_ ? 1 + ActionMoveWest : 1 + ActionTurnLeft;
    }
    ~wumpus_distances_t() { }

    int nrows() const { return nrows_; }
    int ncols() const { return ncols_; }
    int operator[](int i) const { return distances_[i]; }

    void compute_distances(const T &state, bool safe, int p) const {
        _initialize_distances();
        _setup_goal_pos(p);
        _compute_distances(state, safe);
    }

    void print(std::ostream &os) const {
        os << "distances:";
        int dim = nrows_ * ncols_;
        if( !nesw_movements_ ) dim *= 4;
        for( int p = 0; p < dim; ++p )
            os << " " << p << ":" << distances_[p];
        os << std::endl;
    }
};

template<typename T> struct shortest_distance_to_unvisited_cell_t : public Heuristic::heuristic_t<T> {
    const template_problem_t<T> &problem_;
    bool nesw_movements_;
    wumpus_distances_t<T> distances_;
    mutable std::vector<bool> visited_cells_;
    int ncells_;

  public:
    shortest_distance_to_unvisited_cell_t(const template_problem_t<T> &problem, bool nesw_movements)
      : problem_(problem), nesw_movements_(nesw_movements),
        distances_(problem_.nrows(), problem_.ncols(), nesw_movements) {
        assert(nesw_movements_);
        ncells_ = problem_.nrows() * problem_.ncols();
    }
    virtual ~shortest_distance_to_unvisited_cell_t() { }

    void prepare_new_trial() const {
        visited_cells_ = std::vector<bool>(ncells_, false);
    }
    void mark_as_visited(int cell) const {
        visited_cells_[cell] = true;
    }

    virtual float value(const T &s) const {
        if( s.have_gold() ) return 0;
        if( s.in_gold_cell() ) return 1;
        if( s.dead() || !s.in_cave() ) return 1e6;
        distances_.compute_distances(s, false, s.position());
        //std::cout << "position=" << s.position() << std::endl;
        //distances_.print(std::cout);
        int goal_value = distances_[problem_.dim() * problem_.dim() - 1];
        assert(goal_value >= 0);
        return goal_value == INT_MAX ? 1e4 : 1 + goal_value;
    }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const T &s) const { return value(s); }
};

};

};

#undef DEBUG

#endif

