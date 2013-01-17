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

#ifndef BATTLESHIP_WRAPPER_BELIEF
#define BATTLESHIP_WRAPPER_BELIEF

#include "battleship_belief.h"

#include <iostream>

namespace Battleship {

class wrapper_belief_t {
  protected:
    static float prior_water_at_cell_;

  protected:
    int num_hit_segments_;
    int num_uncovered_cells_;
    std::vector<bool> uncovered_cells_;
    mutable bool hit_status_before_torpedo_;
    belief_t *belief_;

  public:
    wrapper_belief_t() {
        num_hit_segments_ = 0;
        num_uncovered_cells_ = 0;
        uncovered_cells_ = std::vector<bool>(base_belief_t::ncells_, false);
        hit_status_before_torpedo_ = false;
        //belief_ = belief_t::allocate();
        belief_ = 0;
    }
    wrapper_belief_t(const wrapper_belief_t &bel) {
        num_hit_segments_ = bel.num_hit_segments_;
        num_uncovered_cells_ = bel.num_uncovered_cells_;
        uncovered_cells_ = bel.uncovered_cells_;
        hit_status_before_torpedo_ = bel.hit_status_before_torpedo_;
        belief_ = belief_t::allocate();
        *belief_ = *bel.belief_;
    }
    wrapper_belief_t(wrapper_belief_t &&bel) {
        num_hit_segments_ = bel.num_hit_segments_;
        num_uncovered_cells_ = bel.num_uncovered_cells_;
        uncovered_cells_ = std::move(bel.uncovered_cells_);
        hit_status_before_torpedo_ = bel.hit_status_before_torpedo_;
        belief_ = bel.belief_;
        bel.belief_ = 0;
    }
    virtual ~wrapper_belief_t() {
        belief_t::deallocate(belief_);
    }

    static void initialize(int nrows,
                           int ncols,
                           int max_ship_size,
                           const int *ship_inventory,
                           bool simple_observations,
                           bool allow_adjacent_ships) {
        belief_t::initialize(nrows,
                             ncols,
                             max_ship_size,
                             ship_inventory,
                             simple_observations,
                             allow_adjacent_ships);

        // calculate prior
        int num_segments = 0;
        for( int i = max_ship_size; i > 0; --i )
            num_segments += i * ship_inventory[i];
        prior_water_at_cell_ = 1.0 - ((float)num_segments / (nrows * ncols));
    }

    size_t hash() const {
        return belief_->hash();
    }

    bool consistent() const {
        return belief_->consistent();
    }
    void clear() {
        belief_->clear();
    }
    void set_initial_configuration() {
        num_hit_segments_ = 0;
        num_uncovered_cells_ = 0;
        uncovered_cells_ = std::vector<bool>(base_belief_t::ncells_, false);
        if( belief_ == 0 ) belief_ = belief_t::allocate();
        belief_->set_initial_configuration();
    }

    int num_covered_cells() const { return base_belief_t::ncells_ - num_uncovered_cells_; }
    int num_uncovered_cells() const { return num_uncovered_cells_; }
    int num_hit_segments() const { return num_hit_segments_; }
    bool hit_status_before_torpedo() const { return hit_status_before_torpedo_; }

    bool is_possible_obs(int action, int obs) const {
        //float p = belief_->ship_probability_at_cell(action, prior_water_at_cell_);
        //return obs == 0 ? p < 1 : p > 0;
        float p = belief_->water_probability_at_cell(action, prior_water_at_cell_);
        return obs == 0 ? p > 0 : p < 1;
    }
    float obs_probability(int action, int obs) const {
        //float p = belief_->ship_probability_at_cell(action, prior_water_at_cell_);
        //return obs == 0 ? 1 - p : p;
        float p = belief_->water_probability_at_cell(action, prior_water_at_cell_);
        return obs == 0 ? p : 1 - p;
    }

    int select_action(bool random_play) const {
        if( random_play ) {
            return belief_->select_random_action();
        } else {
            return belief_->select_action(prior_water_at_cell_);
        }
    }

    bool applicable(int action) const {
        return !uncovered_cells_[action];
    }

    void apply_action(int action) {
        if( !uncovered_cells_[action] ) {
            ++num_uncovered_cells_;
            uncovered_cells_[action] = true;
        }
        if( !consistent() ) { print(std::cout); assert(0); }
        hit_status_before_torpedo_ = belief_->hit_status(action);
        belief_->fire_torpedo(action);
    }
    void update(int action, int obs) {
#if 0
        belief_t b(*belief_);
        b.filter(action, obs);
        if( !b.consistent() ) {
            std::cout << "Fire @ (" << (action % 3) << "," << (action / 3) << "), obs=" << obs << std::endl;
            print(std::cout);
            assert(0);
        }
#endif
        belief_->filter(action, obs);
        //if( !consistent() ) { print(std::cout); }
        if( (obs == 1) && !hit_status_before_torpedo_ ) {
            if( consistent() ) {
                assert(belief_->hit_status(action));
                ++num_hit_segments_;
            }
        }
    }
    void apply_action_and_update(int action, int obs) {
        apply_action(action);
        update(action, obs);
    }

    const wrapper_belief_t& operator=(const wrapper_belief_t &bel) {
        num_hit_segments_ = bel.num_hit_segments_;
        num_uncovered_cells_ = bel.num_uncovered_cells_;
        uncovered_cells_ = bel.uncovered_cells_;
        hit_status_before_torpedo_ = bel.hit_status_before_torpedo_;
        if( belief_ == 0 ) belief_ = belief_t::allocate();
        *belief_ = *bel.belief_;
        return *this;
    }
    bool operator==(const wrapper_belief_t &bel) const {
        return (num_hit_segments_ == bel.num_hit_segments_) &&
               (num_uncovered_cells_ == bel.num_uncovered_cells_) &&
               (uncovered_cells_ == bel.uncovered_cells_) &&
               (belief_ != 0) &&
               (*belief_ == *bel.belief_);
    }
    bool operator!=(const wrapper_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        os << "#hit-segments=" << num_hit_segments_ << std::endl;
        os << "hit-status-before-torpedo=" << hit_status_before_torpedo_ << std::endl;
        belief_->print(os);
    }
};

float wrapper_belief_t::prior_water_at_cell_ = 0;

}; // end of namespace Battleship

inline std::ostream& operator<<(std::ostream &os, const Battleship::wrapper_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

