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

namespace Battleship {

class api_t {
    int nrows_;
    int ncols_;
    const int *ship_inventory_;
    int max_ship_size_;
    int num_ship_segments_;

    bool simple_observations_;
    bool allow_adjacent_ships_;
    bool random_play_;

    mutable std::vector<bool> fired_torpedos_;
    mutable int last_selected_action_;
    mutable int last_obs_;

    problem_t *problem_;
    wrapper_belief_t *belief_;

    std::string policy_name_;
    const Online::Policy::policy_t<wrapper_belief_t> *policy_;

    std::vector<std::pair<const Online::Policy::policy_t<wrapper_belief_t>*, std::string> > bases_;
    std::vector<std::pair<const Heuristic::heuristic_t<wrapper_belief_t>*, std::string> > heuristics_;

    Online::Evaluation::parameters_t eval_pars_;

  public:
    api_t(int nrows, int ncols, const int *ship_inventory, int max_ship_size, bool simple_observations = false, bool allow_adjacent_ships = true, bool random_play = false)
      : nrows_(nrows), ncols_(ncols),
        ship_inventory_(ship_inventory),
        max_ship_size_(max_ship_size),
        simple_observations_(simple_observations),
        allow_adjacent_ships_(allow_adjacent_ships),
        random_play_(random_play) {

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

        // construct heuristic
        admissible_heuristic_t *h = new admissible_heuristic_t(num_ship_segments_);
        heuristics_.push_back(std::make_pair(h, "admissible_heuristic"));
        //heuristic_ = new heuristic_t(1000, nrows_ * ncols_, num_ship_segments_);

        // set base policies and default policy
        base_policy_t base(*problem_);
        bases_.push_back(std::make_pair(base.clone(), "base-policy"));
        base_policy_t random_base(*problem_, true);
        bases_.push_back(std::make_pair(random_base.clone(), "random-base-policy"));
        Online::Policy::random_greedy_t<wrapper_belief_t> greedy(*problem_, *h);
        bases_.push_back(std::make_pair(greedy.clone(), "random-greedy-policy"));
    }
    virtual ~api_t() {
        for( size_t i = 0; i < bases_.size(); ++i )
            delete bases_[i].first;
        delete belief_;
        delete problem_;
    }

    void select_policy(const std::string &base_name, const std::string &policy_type) {
        std::pair<const Online::Policy::policy_t<wrapper_belief_t>*, std::string> p =
          Online::Evaluation::select_policy(*problem_, base_name, policy_type, bases_, heuristics_, eval_pars_);
        policy_ = p.first;
        policy_name_ = p.second;
        std::cout << "api_t: policy=\"" << policy_name_ << "\" selected!" << std::endl;
    }

    void set_policy_parameters(int width, int depth, float par1, float par2) {
        eval_pars_.width_ = width;
        eval_pars_.depth_ = depth;
        eval_pars_.par1_ = par1;
        eval_pars_.par2_ = par2;
    }

    void prepare_new_trial() {
        belief_->set_initial_configuration();
        fired_torpedos_ = std::vector<bool>(nrows_ * ncols_, false);
    }
    int select_action() const {
        last_selected_action_ = (*policy_)(*belief_);
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

