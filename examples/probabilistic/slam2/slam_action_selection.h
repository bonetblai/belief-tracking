/*
 *  Copyright (C) 2015 Universidad Simon Bolivar
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

#ifndef SLAM_ACTION_SELECTION_H
#define SLAM_ACTION_SELECTION_H

#include <cassert>
#include <math.h>
#include "action_selection.h"
#include "cellmap.h"

//#define DEBUG

struct random_slam_policy_t : public action_selection_t<cellmap_t> {
    random_slam_policy_t(const cellmap_t &cellmap) : action_selection_t<cellmap_t>(cellmap) { }
    virtual ~random_slam_policy_t() { }
    virtual int select_action(const tracking_t<cellmap_t> *tracking) const {
        return base_.random_action();
    }
};

struct exploration_slam_policy_t : public action_selection_t<cellmap_t> {
    float map_epsilon_;
    mutable std::set<int> agenda_;
    mutable float *marginals_;

    exploration_slam_policy_t(const cellmap_t &cellmap, float map_epsilon)
      : action_selection_t<cellmap_t>(cellmap), map_epsilon_(map_epsilon) {
        marginals_ = new float[base_.marginals_size_];
    }
    virtual ~exploration_slam_policy_t() {
        delete[] marginals_;
    }

    virtual int select_action(const tracking_t<cellmap_t> *tracking) const {
        // read marginals
        tracking->store_marginals(marginals_);

        // get current position(s) from agenda
        std::vector<coord_t> current_loc;
        std::vector<std::pair<float, int> > map_values;
        tracking->MAP_on_var(marginals_, base_.nloc_, map_values, map_epsilon_);
        current_loc.reserve(map_values.size());
        for( int i = 0; i < int(map_values.size()); ++i ) {
            current_loc.push_back(coord_t(map_values[i].second));
        }
#ifdef DEBUG
        std::cout << std::endl << "policy: current_loc={";
        for( int i = 0; i < int(current_loc.size()); ++i )
            std::cout << current_loc[i] << ",";
        std::cout << "}" << std::endl;
#endif
        assert(!current_loc.empty());

        // remove current position(s) from agenda
        for( int i = 0; i < int(current_loc.size()); ++i ) {
            const coord_t &loc = current_loc[i];
            agenda_.erase(loc.as_index());
        }

        // rebuild agenda if necessary
        if( agenda_.empty() ) {
            build_agenda(tracking);
            if( agenda_.empty() ) { // if agenda still empty, return random action
                //std::cout << "policy: random action" << std::endl;
                return base_.random_action();
            }
        }
#ifdef DEBUG
        std::cout << "policy: agenda={";
        for( std::set<int>::const_iterator it = agenda_.begin(); it != agenda_.end(); ++it )
            std::cout << coord_t(*it) << ",";
        std::cout << "}" << std::endl;
#endif
        assert(!agenda_.empty());

        // select target from agenda and best action(s) towards it
        std::vector<int> candidates;
        for( std::set<int>::const_iterator it = agenda_.begin(); it != agenda_.end(); ++it ) {
            coord_t target(*it);
            for( int i = 0; i < int(current_loc.size()); ++i ) {
                const coord_t &loc = current_loc[i];
                if( target != loc )
                    candidates.push_back(base_.action_for(loc, target));
            }
            if( !candidates.empty() ) break;
        }

        // return a best action
        if( candidates.empty() )
            return base_.random_action();
        else
            return candidates[lrand48() % candidates.size()];
    }

    void build_agenda(const tracking_t<cellmap_t> *tracking) const {
        assert(agenda_.empty());
        std::vector<std::pair<float, int> > map_values;
        for( int loc = 0; loc < base_.nloc_; ++loc ) {
            tracking->MAP_on_var(marginals_, loc, map_values, map_epsilon_);
#if 0
            std::cout << "policy: mapsz[loc=" << loc << "]=" << map_values.size() << ", map={";
            for( int i = 0; i < int(map_values.size()); ++i )
                std::cout << "(" << map_values[i].first << "," << map_values[i].second << "),";
            std::cout << "}" << std::endl;
#endif
            if( map_values.size() > 1 )
                agenda_.insert(loc);
        }
#ifdef DEBUG
        std::cout << "policy: build agenda: sz=" << agenda_.size() << std::endl;
#endif
    }
};

struct murphy_nips99_slam_policy_t : public action_selection_t<cellmap_t> {
    float discount_;
    float epsilon_;
    float map_epsilon_;

    std::vector<std::vector<std::pair<int, float> >*> tr_model_;

    mutable float *reward_;
    mutable float *qvalue_;
    mutable float *value_function_;
    mutable float *marginals_;
    mutable int *best_action_;
    mutable int loc_with_best_reward_;

    mutable std::vector<int> reversed_plan_;

    murphy_nips99_slam_policy_t(const cellmap_t &cellmap, float discount, float epsilon, float map_epsilon)
      : action_selection_t<cellmap_t>(cellmap), discount_(discount), epsilon_(epsilon), map_epsilon_(map_epsilon) {
        reward_ = new float[base_.nloc_];
        qvalue_ = new float[5 * base_.nloc_];
        value_function_ = new float[base_.nloc_];
        marginals_ = new float[base_.marginals_size_];
        best_action_ = new int[base_.nloc_];
        reversed_plan_.reserve(base_.nloc_);

        // calculate tr model
        tr_model_ = std::vector<std::vector<std::pair<int, float> >*>(5 * base_.nloc_, 0);
        for( int action = 0; action < 5; ++action ) {
            for( int loc = 0; loc < base_.nloc_; ++loc ) {
                std::vector<std::pair<int, float> > *model = new std::vector<std::pair<int, float> >;
                for( int nloc = 0; nloc < base_.nloc_; ++nloc ) {
                    float p = base_.probability_tr_loc(action, loc, nloc);
                    if( p != 0 ) model->push_back(std::make_pair(nloc, p));
                }
                tr_model_[action * base_.nloc_ + loc] = model;
            }
        }
    }
    virtual ~murphy_nips99_slam_policy_t() {
        delete[] best_action_;
        delete[] marginals_;
        delete[] value_function_;
        delete[] qvalue_;
        delete[] reward_;
    }

    virtual int select_action(const tracking_t<cellmap_t> *tracking) const {
#if 1
        // read marginals and solve MDP
        tracking->store_marginals(marginals_);

#if 1
        // get current loc
{
        std::vector<std::pair<float, int> > map_values;
        tracking->MAP_on_var(marginals_, base_.nloc_, map_values, map_epsilon_);
        assert(!map_values.empty());
        std::cout << "(cloc=" << coord_t(map_values[0].second) << ",p=" << map_values[0].first << ")" << std::flush;
}
#endif

        solve_mdp(marginals_);

        // calculate best action and return
        int action = greedy_action(&marginals_[base_.variable_offset(base_.nloc_)]);
        std::cout << "(" << base_.action_label(action) << ")" << std::flush;
        //int action = base_.action_for(current_loc, loc_with_best_reward_);
        //if( base_.is_noop_action(action, current_loc) ) std::cout << "RANDOM" << std::flush;
        //return base_.is_noop_action(action, current_loc) ? lrand48() % 4 : action;
        return action == cellmap_t::noop ? lrand48() % 4 : action;
#else
        if( !reversed_plan_.empty() ) {
            int action = reversed_plan_.back();
            assert(action != cellmap_t::noop);
            reversed_plan_.pop_back();
            //std::cout << "#policy: action=" << action << std::endl;
            return action;
        } else {
            //std::cout << "#policy: build reversed plan" << std::endl;
            // read marginals and solve MDP
            tracking->store_marginals(marginals_);
            solve_mdp(marginals_);

            // get current loc
            int current_loc = 0;
            std::vector<std::pair<float, int> > map_values;
            tracking->MAP_on_var(marginals_, base_.nloc_, map_values, map_epsilon_);
            assert(!map_values.empty());
            current_loc = map_values.begin()->second;
            //std::cout << "#policy: current_loc=" << coord_t(current_loc) << ", p=" << map_values.begin()->first << std::endl;

            // fill in reversed plan: path from current loc to best cell
            //int i = 0;
            std::vector<int> plan;
            std::set<int> visited_loc;
            int action = greedy_action(current_loc, 1);
            while( !base_.is_noop_action(action, current_loc) && (visited_loc.find(current_loc) == visited_loc.end()) ) {
                plan.push_back(action);
                visited_loc.insert(current_loc);
                current_loc = base_.sample_loc(current_loc, action); // deterministic actions
                action = greedy_action(current_loc);
            }
            reversed_plan_.insert(reversed_plan_.begin(), plan.rbegin(), plan.rend());

            // return first action in plan
            if( reversed_plan_.empty() ) std::cout << "RANDOM" << std::flush;
            if( reversed_plan_.empty() )
                return lrand48() % 4;
            else
                return select_action(tracking);
        }
#endif
    }

    const std::vector<std::pair<int, float> > &tr_model(int action, int loc) const {
        return *tr_model_[action * base_.nloc_ + loc];
    }

    float normalized_entropy_dist(const float *distribution, int n) const {
        float entropy = 0;
        for( int i = 0; i < n; ++i ) {
            float p = distribution[i];
            if( (p > 0) && (p < 1) ) entropy += p * -log2f(p);
        }
        entropy /= log2f(n);
        if( (entropy < 0) && (entropy + 1e-5 > 0) ) entropy = 0;
        if( (entropy > 1) && (entropy - 1e-5 < 1) ) entropy = 1;
        assert((entropy >= 0) && (entropy <= 1));
        return entropy;
    }
    float normalized_entropy(const float *marginals, int var) const {
        int n = var < base_.nloc_ ? base_.nlabels_ : base_.nloc_;
        return normalized_entropy_dist(&marginals[base_.variable_offset(var)], n);
    }

    void solve_mdp(const float *marginals, float q = -1) const {
        // calculate (normalized) entropy for loc (used to define rewards for entering cells)
        float loc_entropy = normalized_entropy(marginals, base_.nloc_);

        // calculate rewards
        int loc_with_max_entropy = 0;
        float entropy_loc_with_max_entropy = 0;
        loc_with_best_reward_ = -1;
        for( int loc = 0; loc < base_.nloc_; ++loc ) {
            // calculate (normalized) entropies for cells (used to define rewards for entering cells)
            float map_entropy = normalized_entropy(marginals, loc);

            if( map_entropy > entropy_loc_with_max_entropy ) {
                loc_with_max_entropy = loc;
                entropy_loc_with_max_entropy = map_entropy;
            }

            // calculate reward and loc with best reward
            reward_[loc] = loc_entropy * (1 - map_entropy) + (1 - loc_entropy) * map_entropy;
            if( (loc_with_best_reward_ == -1) || (reward_[loc_with_best_reward_] < reward_[loc]) )
                loc_with_best_reward_ = loc;
        }
        std::cout << "(loc-maxH=" << coord_t(loc_with_max_entropy) << ",H=" << entropy_loc_with_max_entropy << ")" << std::flush;

        // solve MDP
        bzero(value_function_, base_.nloc_ * sizeof(float));
        float residual = std::numeric_limits<float>::max();
        while( residual > epsilon_ ) {
            residual = 0;
            for( int loc = 0; loc < base_.nloc_; ++loc ) {
                float value = -1;
                for( int action = 0; action < 5; ++action ) { // action with index 4 is noop
                    const std::vector<std::pair<int, float> > &tr = tr_model(action, loc);
                    float qvalue = 0;
                    for( int i = 0; i < int(tr.size()); ++i ) {
                        int nloc = tr[i].first;
                        float p = tr[i].second;
                        qvalue += p * (reward_[nloc] + discount_ * value_function_[nloc]);
                    }
                    assert(qvalue >= 0);
                    qvalue_[action * base_.nloc_ + loc] = qvalue;

                    if( qvalue > value ) {
                        value = qvalue;
                        best_action_[loc] = action;
                    }
                }
                residual = std::max(fabsf(value_function_[loc] - value), residual);
                value_function_[loc] = value;
            }
        }
        int loc_with_max_value = 0;
        int value_loc_with_max_value = 0;
        for( int loc = 0; loc < base_.nloc_; ++loc ) {
            if( value_function_[loc] > value_loc_with_max_value ) {
                loc_with_max_value = loc;
                value_loc_with_max_value = value_function_[loc];
            }
        }
        std::cout << "(loc-maxV=" << coord_t(loc_with_max_value) << ",V=" << value_loc_with_max_value << ")" << std::flush;
    }

    int greedy_action(int current_loc, float q = -1) const {
        return best_action_[current_loc];
    }

    int greedy_action(const float *marginal) const {
        std::vector<int> best_actions;
        float best_expected_qvalue = 0;
        for( int action = 0; action < 5; ++action ) {
            float expected_qvalue = 0;
            for( int loc = 0; loc < base_.nloc_; ++loc ) {
                const std::vector<std::pair<int, float> > &tr = tr_model(action, loc);
                float qvalue = 0;
                for( int i = 0; i < int(tr.size()); ++i ) {
                    int nloc = tr[i].first;
                    float p = tr[i].second;
                    //qvalue += p * (reward_[nloc]);// + discount_ * value_function_[nloc]);
                    qvalue += p * (reward_[nloc] + discount_ * value_function_[nloc]);
                }
                expected_qvalue += marginal[loc] * qvalue;
            }

            // compute best actions
            if( best_actions.empty() || (expected_qvalue > best_expected_qvalue) ) {
                if( expected_qvalue > best_expected_qvalue )
                    best_actions.clear();
                best_actions.push_back(action);
                best_expected_qvalue = expected_qvalue;
            }
        }
        assert(!best_actions.empty());

        // return best action
        return best_actions[lrand48() % best_actions.size()];
    }
};

#undef DEBUG

#endif

