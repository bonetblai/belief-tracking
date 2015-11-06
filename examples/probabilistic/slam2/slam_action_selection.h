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

#define DEBUG

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

#undef DEBUG

#endif

