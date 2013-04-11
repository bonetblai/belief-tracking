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

#include <dispatcher.h>


namespace Wumpus {

struct abstract_api_t {
    int nrows_;
    int ncols_;
    int npits_;
    int nwumpus_;
    int narrows_;
    bool nesw_movements_;
    int seed_;

    std::string policy_name_;
    Online::Evaluation::parameters_t eval_pars_;

  public:
    abstract_api_t(int nrows, int ncols, int npits, int nwumpus, int narrows, bool nesw_movements)
      : nrows_(nrows), ncols_(ncols), npits_(npits), nwumpus_(nwumpus), narrows_(narrows),
        nesw_movements_(nesw_movements), seed_(0) {
        Random::set_seed(seed_);
    }
    virtual ~abstract_api_t() { }

    void set_seed(int seed) {
        seed_ = seed;
        Random::set_seed(seed_);
        std::cout << "seed=" << seed_ << std::endl;
    }

    void set_policy_parameters(int width, int depth, float par1, float par2) {
        eval_pars_.width_ = width;
        eval_pars_.depth_ = depth;
        eval_pars_.par1_ = par1;
        eval_pars_.par2_ = par2;
    }

    virtual void select_policy(const std::string &base_name, const std::string &policy_type) = 0;
    virtual void prepare_new_trial(int heading, bool diagonal = false) = 0;
    virtual int select_action() const = 0;
    virtual void update(int obs) = 0;
    virtual void apply_action_and_update(int action, int obs) = 0;
    virtual void print(std::ostream &os) const = 0;

    // exact inference
    virtual bool is_world_explored() const = 0;
    virtual bool is_there_an_unvisited_safe_cell() const = 0;
};

template<typename T> struct template_wumpus_api_t : public abstract_api_t {
    template_problem_t<T> *problem_;
    char *sensed_info_;
    T *state_;

    std::vector<std::pair<const Online::Policy::policy_t<T>*, std::string> > bases_;
    std::vector<std::pair<const Heuristic::heuristic_t<T>*, std::string> > heuristics_;

    shortest_distance_to_unvisited_cell_t<T> *heuristic_;
    const Online::Policy::policy_t<T> *policy_;

  public:
    template_wumpus_api_t(int nrows, int ncols, int npits, int nwumpus, int narrows, bool nesw_movements)
      : abstract_api_t(nrows, ncols, npits, nwumpus, narrows, nesw_movements),
        problem_(0), state_(0), heuristic_(0), policy_(0) {
        sensed_info_ = new char[nrows_ * ncols_];

        T::initialize(nrows_, ncols_);
        wumpus_belief_t::initialize(nrows_, ncols_, npits_, nwumpus_);
        moving_wumpus_belief_t::initialize(nrows_, ncols_, npits_, nwumpus_);

        problem_ = new template_problem_t<T>(nrows_, ncols_, npits_, nwumpus_, narrows_, GOAL_IS_HAVE_GOLD, 1e5);

        // set heuristic
        heuristic_ = new shortest_distance_to_unvisited_cell_t<T>(*problem_, nesw_movements_);
        heuristics_.push_back(std::make_pair(heuristic_, "shortest_distance_to_unvisited_cell_heuristic"));


        // set base policies
        Online::Policy::greedy_t<T> greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(greedy.clone(), "greedy_wrt_sduv-heuristic"));
        Online::Policy::random_greedy_t<T> random_greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(random_greedy.clone(), "random-greedy_wrt_sduv-heuristic"));
        Online::Policy::optimistic_greedy_t<T> optimistic_greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(optimistic_greedy.clone(), "optimistic-greedy_wrt_sduv-heuristic"));
        Online::Policy::random_t<T> random(*problem_);
        bases_.push_back(std::make_pair(random.clone(), "random"));
        //wumpus_base_policy_t<T> wumpus(*problem_, nrows_, ncols_, nesw_movements_);
        //bases_.push_back(std::make_pair(wumpus.clone(), "wumpus_base"));
    }
    virtual ~template_wumpus_api_t() {
        delete[] sensed_info_;
        delete heuristic_;
        delete problem_;
    }

    virtual void select_policy(const std::string &base_name, const std::string &policy_type) {
        std::pair<const Online::Policy::policy_t<T>*, std::string> p =
          Online::Evaluation::select_policy(*problem_, base_name, policy_type, bases_, heuristics_, eval_pars_);
        if( p.first == 0 ) {
            std::cout << "Warning: unrecognized policy '" << policy_type << ":" << base_name << std::endl;
        } else {
            policy_ = p.first;
            policy_name_ = p.second;
            //std::cout << "template_wumpus_api_t::policy=" << policy_name_ << std::endl;
        }
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
        Problem::action_t action = (*policy_)(*state_);
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
            if( sensed_info_[cell] == 255 ) {
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

};

#endif

