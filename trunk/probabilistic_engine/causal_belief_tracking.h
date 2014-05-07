/*
 *  Copyright (C) 2014 Universidad Simon Bolivar
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

#ifndef CAUSAL_BELIEF_TRACKING_H
#define CAUSAL_BELIEF_TRACKING_H

#include <iostream>
#include <iomanip>
#include <cassert>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "probabilistic_belief_tracking.h"
#include "joint_distribution.h"

//#define DEBUG

namespace ProbabilisticBeliefTracking {

template<typename T> class beam_t : public joint_distribution_t<T> {
  protected:
    std::string beam_string_;
    std::set<int> variables_;
    std::map<int, int> variable_map_;
    std::vector<int> inverse_variable_map_;

  public:
    beam_t(const std::string &beam_string = "") : beam_string_(beam_string) { }

    const std::string& beam_string() const { return beam_string_; }
    void set_string(const std::string &beam_string) { beam_string_ = beam_string; }

    const std::set<int>& variables() const { return variables_; }
    void set_variables(const std::vector<int> &inverse_variable_map) {
        inverse_variable_map_ = inverse_variable_map;
        variables_.clear();
        variable_map_.clear();
        for( size_t k = 0; k < inverse_variable_map_.size(); ++k ) {
            variables_.insert(inverse_variable_map_[k]);
            variable_map_[inverse_variable_map_[k]] = k;
        }
    }
    int map_variable(int varid) const {
        std::map<int, int>::const_iterator it = variable_map_.find(varid);
        assert(it != variable_map_.end());
        return it->second;
    }
    int inverse_map_variable(int varid) const { return inverse_variable_map_[varid]; }
    void remap_event(BeliefTracking::event_t &event) const {
        for( size_t k = 0; k < event.size(); ++k )
            event[k].first = map_variable(event[k].first);
    }
    bool contains_event_variables(const BeliefTracking::event_t &event) const {
        for( size_t k = 0; k < event.size(); ++k ) {
            if( variables_.find(event[k].first) == variables_.end() )
                return false;
        }
        return true;
    }

    void add_valuation(const BeliefTracking::valuation_t &valuation, T p) {
        int index = joint_distribution_t<T>::encode_valuation(valuation);
        joint_distribution_t<T>::table_[index] += p;
    }
    void project(const BeliefTracking::valuation_t &valuation, float p) {
        BeliefTracking::valuation_t beam_valuation(variables_.size(), 0);
        for( std::set<int>::const_iterator it = variables_.begin(); it != variables_.end(); ++it )
            beam_valuation[map_variable(*it)] = valuation[*it];
        add_valuation(beam_valuation, p);
    }

    const beam_t<T>& operator=(const beam_t<T> &beam) {
        *static_cast<joint_distribution_t<T>*>(this) = static_cast<const joint_distribution_t<T>&>(beam);
        variables_ = beam.variables_;
        variable_map_ = beam.variable_map_;
        inverse_variable_map_ = beam.inverse_variable_map_;
        return *this;
    }

    void print(std::ostream &os, int prec = 2, int decode = true) const {
        os << "beam: id=" << beam_string_ << std::endl;
        os << "beam: vars={";
        for( std::set<int>::const_iterator it = variables_.begin(); it != variables_.end(); ++it )
            os << *it << ",";
        os << "}" << std::endl;
        os << "beam: inv-map={";
        for( int k = 0; k < inverse_variable_map_.size(); ++k )
            os << k << "->" << inverse_variable_map_[k] << ",";
        os << "}" << std::endl;
        joint_distribution_t<T>::print(os, prec, decode);
    }
};

template<typename T> class causal_belief_tracking_t : public probabilistic_belief_tracking_t {
  public:
    struct const_join_iterator {
        // NEED TO FINISH
        bool operator==(const const_join_iterator &it) const {
            // NEED TO FINISH
            return false;
        }
        bool operator!=(const const_join_iterator &it) const { return !(*this == it); }
        const const_join_iterator& operator++() const {
            // NEED TO FINISH
            return *this;
        }
        void get_valuation(BeliefTracking::valuation_t &valuation) const {
            // NEED TO FINISH
        }
    };

  protected:
    std::vector<beam_t<T> > beams_;
    std::set<int> variables_;

    static void clone_beams(std::vector<beam_t<T> > &src, std::vector<beam_t<T> > &dst) {
        dst = src;
    }

  public:
    causal_belief_tracking_t(int nbeams = 0) : beams_(nbeams) { }
    virtual ~causal_belief_tracking_t() { }

    int nbeams() const { return beams_.size(); }
    const std::vector<beam_t<T> >& beams() const { return beams_; }

    void calculate_variables() {
        variables_.clear();
        for( int k = 0; k < beams_.size(); ++k )
            variables_.insert(beams_[k].variables().begin(), beams_[k].variables().end());
    }

    void clear() {
        for( size_t k = 0; k < beams_.size(); ++k )
            beams_[k].clear();
    }
    void set_uniform_distribution() {
        for( size_t k = 0; k < beams_.size(); ++k )
            beams_[k].set_uniform_distribution();
    }

    const_join_iterator begin_join() const {
        // NEED TO FINISH
        return const_join_iterator();
    }
    const_join_iterator end_join() const {
        // NEED TO FINISH
        return const_join_iterator();
    }

    void project(const BeliefTracking::valuation_t &valuation, float p, std::vector<beam_t<T> > &nbeams) const {
        std::cout << "begin-project: val=" << valuation << ", p=" << p << std::endl;
        if( p > 0 ) {
            for( int k = 0; k < beams_.size(); ++k )
                nbeams[k].project(valuation, p);
        }
        std::cout << "end-project" << std::endl;
    }

    virtual float probability(const BeliefTracking::event_t &event) const {
        // event must fall within some beam. First thing to do is to
        // find the beam for the event. Then, the probability of the
        // event is the probability assigned by the beam.
        int beam_index = -1;
        for( int k = 0; k < beams_.size(); ++k ) {
            if( beams_[k].contains_event_variables(event) ) {
                beam_index = k;
                break;
            }
        }
        if( beam_index == -1 ) return -1;

        BeliefTracking::event_t remapped_event(event);
        beams_[beam_index].remap_event(remapped_event);
        return beams_[beam_index].probability(remapped_event);
    }

    virtual void progress_and_filter(int action, int obs,
                                     BeliefTracking::belief_tracking_t::det_progress_func_t progress,
                                     BeliefTracking::belief_tracking_t::filter_func_t filter)
    {
        std::vector<beam_t<T> > nbeams;
        clone_beams(beams_, nbeams);
        BeliefTracking::valuation_t valuation(variables_.size(), 0);
        for( const_join_iterator it = begin_join(); it != end_join(); ++it ) {
            it.get_valuation(valuation);
            (this->*progress)(action, valuation);
            float p = (this->*filter)(obs, action, valuation);
            project(valuation, p, nbeams);
        }
        clone_beams(nbeams, beams_);
    }

    virtual void print(std::ostream &os) const {
        for( int k = 0; k < beams_.size(); ++k )
            beams_[k].print(os);
    }

};

}; // end of namespace ExactDistribution

#undef DEBUG

#endif

