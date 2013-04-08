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

#ifndef WUMPUS_PROBLEM_H
#define WUMPUS_PROBLEM_H

#include "wumpus_state.h"

#include <problem.h>
#include <heuristic.h>
#include <policy.h>

#include <vector>
#include <set>

#define GOAL_IS_HAVE_GOLD   0
#define GOAL_IS_EXIT        1


namespace Wumpus {

namespace Diagonal {

template<typename T> class template_problem_t : public Problem::problem_t<T> {
  protected:
    int dim_;
    const T init_;
    const Heuristic::heuristic_t<T> *heuristic_;

  public:
    template_problem_t(int dim, float dead_end_value = 1e5)
      : Problem::problem_t<T>(1.0, dead_end_value), dim_(dim), heuristic_(0) {
        const_cast<T&>(init_).set_initial_configuration();
    }
    virtual ~template_problem_t() { }

    int dim() const { return dim_; }
    int nrows() const { return dim_; }
    int ncols() const { return dim_; }

    void set_heuristic(const Heuristic::heuristic_t<T> *heuristic) {
        heuristic_ = heuristic;
    }
    float heuristic(const T &s) const { return heuristic_ == 0 ? 0 : heuristic_->value(s); }

    virtual Problem::action_t number_actions(const T &s) const {
        return 1 + ActionExit;
    }
    virtual bool applicable(const T &s, Problem::action_t a) const {
        return s.applicable(a);
    }
    virtual const T& init() const {
        return init_;
    }
    virtual bool terminal(const T &s) const {
        return s.have_gold();
    }
    virtual bool dead_end(const T &s) const {
        return (!s.terminal() && !s.in_cave()) || s.dead() || (heuristic(s) >= 10000);
    }
    virtual float cost(const T &s, Problem::action_t a) const {
        return (a == ActionExit) && !s.have_gold() ? 1e4 : 1;
    }

    virtual void next(const T &s, Problem::action_t a, std::vector<std::pair<T, float> > &outcomes) const {
        //std::cout << "action=" << a << std::endl;
        //std::cout << "state=" << s;
        assert(s.applicable(a));
        T next_a = s;
        next_a.apply(a);
        //std::cout << "next_state=" << next_a;

        // 105 is max possible number of obs because we limit number
        // of wumpus to 1. The general formula for n wumpus is
        // max_nobs = 1 + 4 * \sum_{k = 0} ^ n \binom{25}{k}.
        int possible_obs[105], nobs = 0;
        for( int obs = 0; obs < 105; ++obs ) {
            if( next_a.possible_obs(a, obs) ) {
                possible_obs[nobs++] = obs;
            }
        }
        assert(nobs > 0);

        outcomes.clear();
        outcomes.reserve(nobs);
        float p = 1.0 / (float)nobs;
        for( int i = 0; i < nobs; ++i ) {
            int obs = possible_obs[i];
            T next_ao = next_a;
            next_ao.update(a, obs);
            assert(!next_ao.inconsistent());
            //std::cout << "obs=" << obs << " is consistent" << std::endl;
            //std::cout << "next_ao: " << next_ao;

            bool found = false;
            for( int j = 0, jsz = outcomes.size(); j < jsz; ++j ) {
                if( outcomes[j].first == next_ao ) {
                    found = true;
                    outcomes[j].second += p;
                    break;
                }
            }

            if( !found ) {
                outcomes.push_back(std::make_pair(next_ao, p));
            }
        }
    }

    virtual void print(std::ostream &os) const { }
};

// template instantiation
typedef template_problem_t<state_t> problem_t;

};

};

template<typename T> inline std::ostream& operator<<(std::ostream &os, const Wumpus::Diagonal::template_problem_t<T> &p) {
    p.print(os);
    return os;
}


namespace Wumpus {

namespace Diagonal {

class hidden_state_t {
    int dim_;
    int pos_;
    int gold_pos_;
    bool have_gold_;
    bool dead_;
    std::vector<int> wumpus_;

  public:
    hidden_state_t(int dim) : dim_(dim), pos_(0), gold_pos_(-1), have_gold_(false), dead_(false) { }
    ~hidden_state_t() { }

    int position() const { return pos_; }
    int gold_position() const { return gold_pos_; }
    bool have_gold() const { return have_gold_; }
    bool dead() const { return dead_; }

    void sample(int pos) {
        pos_ = pos;
        have_gold_ = false;
        dead_ = false;
        gold_pos_ = dim_ * dim_ - 1;

        wumpus_ = std::vector<int>(dim_ * dim_, 0);
        for( int d = 0; d < dim_ - 2; ++d ) {
            if( Random::uniform(2) ) {
                wumpus_[(2 + d) * dim_ + (1 + d)] = 1;
            } else {
                wumpus_[(1 + d) * dim_ + (2 + d)] = 1;
            }
        }
    }

    int get_obs() {
        if( pos_ == OutsideCave ) return 0;
        int breeze = 0; //num_surrounding_objs(pits_);
        int stench = num_surrounding_objs(wumpus_);
        int glitter = gold_pos_ == pos_ ? 1 : 0;
        if( breeze == 9 ) {
            return Fell;
        } else if( stench == 9 ) {
            return Eaten;
        } else {
            int obs = 0;
            obs += glitter > 0 ? Glitter : 0;
            obs += breeze > 0 ? Breeze : 0;
            obs += stench > 0 ? Stench : 0;
            return obs;
        }
    }

    bool applicable(int action) const { assert(0); return false; }
    void apply(int action) { assert(0); }

    int apply_action_and_get_obs(int action) {
        assert(!dead_);
        apply(action);
        dead_ = (pos_ != OutsideCave) && wumpus_[pos_];
        return get_obs();
    }

    int num_surrounding_objs(std::vector<int> &objs) const {
        if( objs[pos_] == 1 ) return 9;

        int row = pos_ / dim_;
        int col = pos_ % dim_;

        int num = 0;
        for( int drow = -1; drow < 2; ++drow ) {
            int nrow = row + drow;
            if( (nrow < 0) || (nrow >= dim_) ) continue;
            for( int dcol = -1; dcol < 2; ++dcol ) {
                if( (drow != 0) && (dcol != 0) ) continue;
                int ncol = col + dcol;
                if( (ncol < 0) || (ncol >= dim_) ) continue;
                num += objs[nrow * dim_ + ncol];
            }
        }
        assert((0 <= num) && (num < 9));
        return num;
    }
};

};

};

#endif

