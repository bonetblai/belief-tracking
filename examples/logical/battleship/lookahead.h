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

#ifndef LOOKAHEAD_H
#define LOOKAHEAD_H

#include "battleship_wrapper_belief.h"

#include <problem.h>
#include <policy.h>
#include <heuristic.h>

#include <cassert>
#include <iostream>

namespace Battleship {

class problem_t : public Problem::problem_t<wrapper_belief_t> {
  protected:
    int nrows_;
    int ncols_;
    int ncells_;
    int num_ship_segments_;
    wrapper_belief_t *initial_belief_;

  public:
    problem_t(int nrows, int ncols, int num_ship_segments, float dead_end_value = 1e5)
      : Problem::problem_t<wrapper_belief_t>(1.0, dead_end_value),
        nrows_(nrows), ncols_(ncols), num_ship_segments_(num_ship_segments) {
        ncells_ = nrows_ * ncols_;
        initial_belief_ = new wrapper_belief_t;
        initial_belief_->set_initial_configuration();
    }
    virtual ~problem_t() { delete initial_belief_; }

    int nrows() const { return nrows_; }
    int ncols() const { return ncols_; }
    int ncells() const { return ncells_; }
    int num_ship_segments() const { return num_ship_segments_; }

    virtual Problem::action_t number_actions(const wrapper_belief_t &s) const {
        return ncells_;
    }
    virtual bool applicable(const wrapper_belief_t &s, Problem::action_t a) const {
        return s.applicable(a);
    }
    virtual const wrapper_belief_t& init() const {
        return *initial_belief_;
    }
    virtual bool terminal(const wrapper_belief_t &s) const {
        return (s.num_hit_segments() == num_ship_segments_) ||
               (s.num_uncovered_cells() == ncells_);
    }
    virtual bool dead_end(const wrapper_belief_t &s) const {
        return !s.consistent();
    }
    virtual float cost(const wrapper_belief_t &s, Problem::action_t a) const {
        return 1;
    }

    virtual void next(const wrapper_belief_t &s, Problem::action_t a,
                      std::vector<std::pair<wrapper_belief_t, float> > &outcomes) const {

        // calculate effect of the action
        assert(s.consistent());
        wrapper_belief_t next_a = s;
        next_a.apply_action(a);

        // calculate possible observations and filterings
        outcomes.clear();
        outcomes.reserve(2);
        float p = next_a.obs_probability(a, 0);
        for( int obs = 0; obs < 2; ++obs ) {
            float q = obs == 0 ? p : 1 - p;
            if( q > 0 ) {
                wrapper_belief_t next_ao = next_a;
                next_ao.update(a, obs);
                if( next_ao.consistent() ) {
                    outcomes.push_back(std::make_pair(next_ao, q));
                }
            }
        }
        assert(!outcomes.empty());

        // normalize probabilities
        if( outcomes.size() == 1 ) {
            outcomes[0].second = 1;
        } else {
            assert(outcomes[0].second + outcomes[1].second == 1);
        }
    }

    virtual void print(std::ostream &os) const { }
};

class base_policy_t : public Online::Policy::policy_t<wrapper_belief_t> {
  protected:
    bool random_policy_;

  public:
    base_policy_t(const Problem::problem_t<wrapper_belief_t> &problem, bool random_policy = false)
      : Online::Policy::policy_t<wrapper_belief_t>(problem), random_policy_(random_policy) { }
    virtual ~base_policy_t() { }

    virtual Problem::action_t operator()(const wrapper_belief_t &s) const {
        return s.select_action(random_policy_);
    }
    virtual const Online::Policy::policy_t<wrapper_belief_t>* clone() const {
        return new base_policy_t(Online::Policy::policy_t<wrapper_belief_t>::problem(), random_policy_);
    }
    virtual void print_stats(std::ostream &os) const { }
};

class admissible_heuristic_t : public Heuristic::heuristic_t<wrapper_belief_t> {
    int num_ship_segments_;
  public:
    admissible_heuristic_t(int num_ship_segments)
      : num_ship_segments_(num_ship_segments) { }
    virtual ~admissible_heuristic_t() { }
    virtual float value(const wrapper_belief_t &s) const {
        return num_ship_segments_ - s.num_hit_segments();
    }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
};

class heuristic_t : public Heuristic::heuristic_t<wrapper_belief_t> {
    int trials_;
    int ncells_;
    int num_ship_segments_;
    float *values_;
  public:
    heuristic_t(int trials, int ncells, int num_ship_segments)
      : trials_(trials), ncells_(ncells), num_ship_segments_(num_ship_segments) {
        compute_table();
    }
    virtual ~heuristic_t() { }
    virtual float value(const wrapper_belief_t &s) const {
        int n = s.num_covered_cells();
        int m = num_ship_segments_ - s.num_hit_segments();
        assert(n > 0);
        return m == 0 ? 0 : values_[(n - 1) * ncells_ + (m - 1)];
    }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }

    void compute_table() {
        std::cout << "computing table for heuristic values... " << std::flush;
        values_ = new float[ncells_ * ncells_];
        char *array = new char[ncells_];
        for( int n = ncells_; n > 0; --n ) {
            for( int m = n; m > 0; --m ) {
                float value = monte_carlo(n, m, array);
                values_[(n - 1) * ncells_ + (m - 1)] = value;
            }
        }
        delete[] array;
        std::cout << "done!" << std::endl;
    }

    float monte_carlo(int n, int m, char *array) const {
        assert(n >= m);
        assert(n <= ncells_);
        memset(array, 0, n * sizeof(char));
        int sum_steps = 0;
        for( int t = 0; t < trials_; ++t ) {
            for( int i = 0; i < m; ++i ) array[i] = 1;
            int hits = 0;
            for( int steps = 0; hits < m; ++steps, ++sum_steps ) {
                assert(steps < n);
                int i = lrand48() % (n - steps);
                if( array[i] == 1 ) ++hits;
                array[i] = array[n - steps - 1];
                array[n - steps - 1] = 0;
            }
        }
        float value = (float)sum_steps / (float)trials_;
        //std::cout << "value=" << value << std::endl;
        return value;
    }
};

}; // end of namespace Battleship

inline std::ostream& operator<<(std::ostream &os, const Battleship::problem_t &p) {
    p.print(os);
    return os;
}


#endif

