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

#include <iostream>
#include <vector>

#include <dispatcher.h>


namespace Wumpus {

namespace Diagonal {

struct abstract_api_t {
    int dim_;
    int seed_;

    std::string policy_name_;
    Online::Evaluation::parameters_t eval_pars_;

  public:
    abstract_api_t(int dim) : dim_(dim), seed_(0) {
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
    virtual void prepare_new_trial(int heading) = 0;
    virtual int select_action() const = 0;
    virtual void update(int obs) = 0;
    virtual void apply(int action) = 0;
    virtual void apply_action_and_update(int action, int obs) = 0;
    virtual void print(std::ostream &os) const = 0;
};

template<typename T> struct template_wumpus_api_t : public abstract_api_t {
    template_problem_t<T> *problem_;
    T *state_;

    std::vector<std::pair<const Online::Policy::policy_t<T>*, std::string> > bases_;
    std::vector<std::pair<const Heuristic::heuristic_t<T>*, std::string> > heuristics_;

    shortest_distance_to_unvisited_cell_t<T> *heuristic_;
    const Online::Policy::policy_t<T> *policy_;

  public:
    template_wumpus_api_t(int dim)
      : abstract_api_t(dim), problem_(0), state_(0), heuristic_(0), policy_(0) {

        T::initialize(dim_);
        wumpus_belief_t::initialize(dim_);
        problem_ = new template_problem_t<T>(dim_, 1e5);

        // set heuristic using NESW movements
        heuristic_ = new shortest_distance_to_unvisited_cell_t<T>(*problem_, true);
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
    }
    virtual ~template_wumpus_api_t() {
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

    virtual void prepare_new_trial(int heading) {
        if( state_ ) delete state_;
        state_ = new T(0, dim_ * dim_ - 1);
        state_->set_initial_configuration();
        heuristic_->prepare_new_trial();
        heuristic_->mark_as_visited(0);
    }

    virtual int select_action() const {
        Problem::action_t action = (*policy_)(*state_);
        assert(action != Problem::noop);
        assert(state_->applicable(action));
        return action;
    }

    virtual void update(int obs) {
        state_->update(-1, obs);
    }
    virtual void apply(int action) {
        state_->apply(action);
        int p = state_->position();
        heuristic_->mark_as_visited(p);
    }
    virtual void apply_action_and_update(int action, int obs) {
        state_->apply_action_and_update(action, obs);
        int p = state_->position();
        heuristic_->mark_as_visited(p);
    }

    virtual void print(std::ostream &os) const {
        os << *state_;
    }
};

typedef template_wumpus_api_t<state_t> wumpus_api_t;

};

};

#endif

