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
    float epsilon_;
    mutable std::set<int> agenda_;
    mutable float *marginals_;

    exploration_slam_policy_t(const cellmap_t &cellmap, float epsilon)
      : action_selection_t<cellmap_t>(cellmap), epsilon_(epsilon) {
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
        tracking->MAP_on_var(marginals_, base_.nloc_, map_values, epsilon_);
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
            tracking->MAP_on_var(marginals_, loc, map_values, epsilon_);
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
    mutable float *reward_;
    mutable float *value_function_;
    mutable float *marginals_;

    murphy_nips99_slam_policy_t(const cellmap_t &cellmap, float discount, float epsilon)
      : action_selection_t<cellmap_t>(cellmap), discount_(discount), epsilon_(epsilon) {
        reward_ = new float[base_.nloc_];
        value_function_ = new float[base_.nloc_];
        marginals_ = new float[base_.marginals_size_];
    }
    virtual ~murphy_nips99_slam_policy_t() {
        delete[] marginals_;
        delete[] value_function_;
        delete[] reward_;
    }

    virtual int select_action(const tracking_t<cellmap_t> *tracking) const {
        // read marginals and solve MDP
        tracking->store_marginals(marginals_);
        solve_mdp(marginals_);

        // get current loc
        int current_loc = 0;
        std::vector<std::pair<float, int> > map_values;
        tracking->MAP_on_var(marginals_, base_.nloc_, map_values, .001);
        assert(!map_values.empty());
        current_loc = map_values.begin()->second;
#ifdef DEBUG
        std::cout << "#policy: current_loc=" << coord_t(current_loc) << ", p=" << map_values.begin()->first << std::endl;
#endif

        // get best action on current loc
        int action = greedy_action(current_loc);
#ifdef DEBUG
        std::cout << "#policy: action=" << action << std::endl;
#endif
        return action;
    }

    float normalized_entropy_dist(const float *distribution, int n) const {
        float entropy = 0;
        for( int i = 0; i < n; ++i ) {
            float p = distribution[i];
            if( (p > 0) && (p < 1) ) entropy += p * -log2f(p);
        }
        return entropy / log2f(n);
    }
    float normalized_entropy(const float *marginals, int var) const {
        int n = var < base_.nloc_ ? base_.nlabels_ : base_.nloc_;
        return normalized_entropy_dist(&marginals[base_.variable_offset(var)], n);
    }

    void solve_mdp(const float *marginals) const {
        // calculate (normalized) entropy for loc that is used to define rewards for entering cells
        float loc_entropy = normalized_entropy(marginals, base_.nloc_);
        assert((loc_entropy >= 0) && (loc_entropy <= 1));

        // calculate rewards
        for( int loc = 0; loc < base_.nloc_; ++loc ) {
            // calculate (normalized) entropies for cells used to define rewards for entering cells
            float h = normalized_entropy(marginals, loc);
            assert((h >= 0) && (h <= 1));
            //std::cout << "#h[" << coord_t(loc) << "]=" << h << std::endl;

            // calculate reward
            reward_[loc] = loc_entropy * (1 - h) + (1 - loc_entropy) * h;
            assert((reward_[loc] >= 0) && (reward_[loc] <= 1));
            //std::cout << "#reward[" << coord_t(loc) << "]=" << reward_[loc] << std::endl;
        }
        //std::cout << "policy: loc-entropy=" << loc_entropy << ", min-h=" << min_h << ", loc=" << coord_t(best_h) << ", reward=" << reward_[best_h] << std::endl;

        // solve MDP
        bzero(value_function_, base_.nloc_ * sizeof(float));
        float residual = std::numeric_limits<float>::max();
        while( residual > epsilon_ ) {
            residual = 0;
            for( int loc = 0; loc < base_.nloc_; ++loc ) {
                float value = 0;
                for( int action = 0; action < 5; ++action ) { // extra action (id=4)
                    float qvalue = 0;
                    for( int nloc = 0; nloc < base_.nloc_; ++nloc ) {
                        float p = base_.probability_tr_loc(action, loc, nloc);
                        qvalue += p * (reward_[nloc] + discount_ * value_function_[nloc]);
                    }
                    assert(qvalue >= 0);
                    value = std::max(value, qvalue);
                }
                residual = std::max(fabsf(value_function_[loc] - value), residual);
                value_function_[loc] = value;
            }
            //std::cout << "residual=" << residual << std::endl;
        }
        for( int loc = 0; loc < base_.nloc_; ++loc )
            ;//std::cout << "#V[" << coord_t(loc) << "]=" << value_function_[loc] << std::endl;
    }

    int greedy_action(int current_loc) const {
        int best_action = -1;
        float best_value = -1;
        for( int action = 0; action < 4; ++action ) {
            float qvalue = 0;
            for( int nloc = 0; nloc < base_.nloc_; ++nloc ) {
                float p = base_.probability_tr_loc(action, current_loc, nloc);
                qvalue += p * (reward_[nloc] + discount_ * value_function_[nloc]);
            }
            assert(qvalue >= 0);
            if( qvalue > best_value ) {
                best_action = action;
                best_value = qvalue;
            }
        }
        assert(best_action >= 0);
        return best_action;
    }
};

#undef DEBUG

#endif

