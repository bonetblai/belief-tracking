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

#ifndef WUMPUS_API_H
#define WUMPUS_API_H

#include "wumpus_problem.h"
#include "base_policy.h"
#include "exact_inference.h"

#include <iostream>
#include <vector>


namespace Wumpus {

class abstract_api_t {
  protected:
    int nrows_;
    int ncols_;
    int npits_;
    int nwumpus_;
    int narrows_;
    bool nesw_movements_;
    bool moving_;
    int seed_;

  public:
    abstract_api_t(int nrows, int ncols, int npits, int nwumpus, int narrows, bool nesw_movements, bool moving)
      : nrows_(nrows), ncols_(ncols), npits_(npits), nwumpus_(nwumpus), narrows_(narrows),
        nesw_movements_(nesw_movements), moving_(moving), seed_(0) {
        Random::set_seed(seed_);
    }
    virtual ~abstract_api_t() { }

    void set_seed(int seed) {
        seed_ = seed;
        Random::set_seed(seed_);
        std::cout << "seed=" << seed_ << std::endl;
    }

    bool is_moving() const { return moving_; }

    virtual void select_policy(const std::string &request) = 0;
    virtual void prepare_new_trial(int heading, bool diagonal = false) = 0;
    virtual int select_action() const = 0;
    virtual void update(int obs) = 0;
    virtual void apply_action_and_update(int action, int obs) = 0;
    virtual void print(std::ostream &os) const = 0;
    virtual std::string current_policy() const = 0;

    // exact inference
    virtual bool is_world_explored() const = 0;
    virtual bool is_there_an_unvisited_safe_cell() const = 0;
};

template<typename T> struct template_wumpus_api_t : public abstract_api_t {
  protected:
    template_problem_t<T> *problem_;
    char *sensed_info_;
    T *state_;

    const Online::Policy::policy_t<T> *current_policy_;
    Dispatcher::dispatcher_t<T> dispatcher_;
    shortest_distance_to_unvisited_cell_t<T> *heuristic_;

  public:
    template_wumpus_api_t(int nrows, int ncols, int npits, int nwumpus, int narrows, bool nesw_movements, bool moving)
      : abstract_api_t(nrows, ncols, npits, nwumpus, narrows, nesw_movements, moving),
        problem_(0), state_(0), current_policy_(0) {
        sensed_info_ = new char[nrows_ * ncols_];

        T::initialize(nrows_, ncols_);
        wumpus_belief_t::initialize(nrows_, ncols_, npits_, nwumpus_);
        moving_wumpus_belief_t::initialize(nrows_, ncols_, npits_, nwumpus_);

        problem_ = new template_problem_t<T>(nrows_, ncols_, npits_, nwumpus_, narrows_, GOAL_IS_HAVE_GOLD, 1e7);

        // create heuristic
        heuristic_ = new shortest_distance_to_unvisited_cell_t<T>(*problem_, nesw_movements_);
        dispatcher_.insert_heuristic(heuristic_->name(), heuristic_);
    }
    virtual ~template_wumpus_api_t() {
        delete[] sensed_info_;
        delete problem_;
    }

    virtual void select_policy(const std::string &request) {
        dispatcher_.create_request(*problem_, "policy", request);
        Online::Policy::policy_t<T> *p = dispatcher_.fetch_policy(request);
        if( p != 0 ) current_policy_ = p;
    }

    virtual void prepare_new_trial(int heading, bool diagonal = false) {
        memset(sensed_info_, 255, nrows_ * ncols_);
        if( state_ ) delete state_;
        state_ = new T(0, heading);
        state_->set_initial_configuration(diagonal);
        heuristic_->prepare_new_trial();
        heuristic_->mark_as_visited(0);
    }

    virtual int select_action() const {
        Problem::action_t action = (*current_policy_)(*state_);
        assert(action != Problem::noop);
        assert(state_->applicable(action));
        //std::cout << "pos=(" << (state_->position() % ncols_) << ","
        //          << (state_->position() / ncols_) << ")"
        //          << ", heading=" << heading_name(state_->heading())
        //          << ": action=" << action
        //          << std::endl;
        return action;
    }

    virtual void update(int obs) {
        state_->update(-1, obs);
        update_sensed_info(state_->position(), obs);
        //std::cout << *state_;
    }
    virtual void apply_action_and_update(int action, int obs) {
        state_->apply_action_and_update(action, obs);
        int p = state_->position();// + (state_->position() << 2);
        heuristic_->mark_as_visited(p);
        update_sensed_info(state_->position(), obs);
        //std::cout << *state_;
    }

    virtual void print(std::ostream &os) const {
        //os << *state_;
        //os << "heuristic=" << heuristic_->value(*state_) << std::endl;
    }

    virtual std::string current_policy() const {
        return current_policy_ == 0 ? std::string("no-such-policy") : current_policy_->name();
    }

    // exact inference
    void update_sensed_info(int pos, int obs) {
        sensed_info_[pos] = obs;
    }

    virtual bool is_world_explored() const {
        exact_inference_t inference(nrows_, ncols_);
        return inference.is_world_explored(sensed_info_);
    }

    virtual bool is_there_an_unvisited_safe_cell() const {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            if( sensed_info_[cell] == (char)255 ) {
                // this cell is unvisited
                if( state_->no_hazard_at(cell) )
                    return true;
            }
        }
        return false;
    }
};

typedef template_wumpus_api_t<state_t> wumpus_api_t;
typedef template_wumpus_api_t<moving_state_t> moving_wumpus_api_t;
typedef template_wumpus_api_t<moving2_state_t> moving2_wumpus_api_t;

};

#endif

