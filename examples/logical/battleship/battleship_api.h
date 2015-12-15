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

#ifndef BATTLESHIP_API
#define BATTLESHIP_API

#include "battleship_wrapper_belief.h"
#include "lookahead.h"

#include <dispatcher.h>

#include <stdlib.h>
#include <cassert>
#include <iostream>
#include <vector>

#ifdef __clang__
#pragma clang diagnostic ignored "-Wunused-private-field"
#endif


namespace Battleship {

class api_t {
    int nrows_;
    int ncols_;
    const int *ship_inventory_;
    int max_ship_size_;
    int num_ship_segments_;

    bool simple_observations_;
    bool allow_adjacent_ships_;

    mutable std::vector<bool> fired_torpedos_;
    mutable int last_selected_action_;
    mutable int last_obs_;

    problem_t *problem_;
    wrapper_belief_t *belief_;

    const Online::Policy::policy_t<wrapper_belief_t> *current_policy_;
    Dispatcher::dispatcher_t<wrapper_belief_t> dispatcher_;

  public:
    api_t(int nrows, int ncols, const int *ship_inventory, int max_ship_size, bool simple_observations = false, bool allow_adjacent_ships = true)
      : nrows_(nrows), ncols_(ncols),
        ship_inventory_(ship_inventory),
        max_ship_size_(max_ship_size),
        simple_observations_(simple_observations),
        allow_adjacent_ships_(allow_adjacent_ships) {

        num_ship_segments_ = 0;
        for( int d = 1; d <= max_ship_size_; ++d )
            num_ship_segments_ += d * ship_inventory_[d];

        wrapper_belief_t::initialize(nrows_,
                                     ncols_,
                                     max_ship_size_,
                                     ship_inventory_,
                                     simple_observations_,
                                     allow_adjacent_ships_);

        problem_ = new problem_t(nrows_, ncols_, num_ship_segments_);
        belief_ = new wrapper_belief_t();
        current_policy_ = 0;

        // construct heuristic
        admissible_heuristic_t *h1 = new admissible_heuristic_t(*problem_, num_ship_segments_);
        dispatcher_.insert_heuristic(h1->name(), h1);
        heuristic_t *h2 = new heuristic_t(*problem_, 1000, nrows_ * ncols_, num_ship_segments_);
        dispatcher_.insert_heuristic(h2->name(), h2);

        // set base policies and default policy
        base_policy_t *p1 = new base_policy_t(*problem_);
        dispatcher_.insert_policy(p1->name(), p1);
        base_policy_t *p2 = new base_policy_t(*problem_, true);
        Online::Policy::random_greedy_t<wrapper_belief_t> *p3 = new Online::Policy::random_greedy_t<wrapper_belief_t>(*problem_, *h1, true);
        dispatcher_.insert_policy(p3->name(), p3);
        current_policy_ = p3;
    }
    virtual ~api_t() {
        delete belief_;
        delete problem_;
        wrapper_belief_t::finalize();
    }

    void select_policy(const std::string &request) {
        dispatcher_.create_request(*problem_, "policy", request);
        Online::Policy::policy_t<wrapper_belief_t> *p = dispatcher_.fetch_policy(request);
        if( p != 0 ) current_policy_ = p;
    }
    const Online::Policy::policy_t<wrapper_belief_t>* current_policy() const {
        return current_policy_;
    }

    void prepare_new_trial() {
        belief_->set_initial_configuration();
        fired_torpedos_ = std::vector<bool>(nrows_ * ncols_, false);
    }
    int select_action() const {
        last_selected_action_ = (*current_policy_)(*belief_);
        assert(belief_->applicable(last_selected_action_));
        return last_selected_action_;
    }
    void update(int obs) {
        apply_action_and_update(last_selected_action_, obs);
    }
    void apply_action_and_update(int action, int obs) {
        if( fired_torpedos_[action] ) {
            std::cout << "REPEATED: fire @ cell(" << (action % ncols_) << "," << (action / ncols_) << ") already done!" << std::endl;
        }
        last_obs_ = obs;
        assert(last_selected_action_ == action);
        belief_->apply_action_and_update(action, obs);
        fired_torpedos_[action] = true;
    }

    void print(std::ostream &os) const {
        belief_->print(os);
    }
};

}; // end of namespace Battleship

#endif

