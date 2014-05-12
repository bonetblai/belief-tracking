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
#include <limits.h>
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
        std::cout << "variables for " << beam_string_ << " ={";
        inverse_variable_map_ = inverse_variable_map;
        variables_.clear();
        variable_map_.clear();
        for( size_t k = 0; k < inverse_variable_map_.size(); ++k ) {
            variables_.insert(inverse_variable_map_[k]);
            variable_map_[inverse_variable_map_[k]] = k;
            std::cout << k << "->" << inverse_variable_map[k] << ",";
        }
        std::cout << "}" << std::endl;
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

    bool is_consistent_with_beam_valuation(const BeliefTracking::valuation_t &valuation) const {
        for( int k = 0; k < joint_distribution_t<T>::nvars_; ++k ) {
            int varid = inverse_map_variable(k);
            if( valuation[varid] != joint_distribution_t<T>::valuation_[k] )
                return false;
        }
        return true;
    }

    void project(const BeliefTracking::valuation_t &valuation, float p) {
        BeliefTracking::valuation_t beam_valuation(variables_.size(), 0);
        for( std::set<int>::const_iterator it = variables_.begin(); it != variables_.end(); ++it )
            beam_valuation[map_variable(*it)] = valuation[*it];
        joint_distribution_t<T>::add_valuation(beam_valuation, p);
    }

    const beam_t<T>& operator=(const beam_t<T> &beam) {
        *static_cast<joint_distribution_t<T>*>(this) = static_cast<const joint_distribution_t<T>&>(beam);
        beam_string_ = beam.beam_string_;
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
    struct const_join_iterator_type1 {
        T probability_;
        BeliefTracking::valuation_t *valuation_;
        const causal_belief_tracking_t *cbt_;

        int n_;
        int *indices_;
        bool overflow_;
        mutable bool invalid_;

        const_join_iterator_type1(const causal_belief_tracking_t *cbt = 0, int n = 0, bool overflow = false)
          : probability_(0), valuation_(0), cbt_(cbt), n_(n), indices_(0), overflow_(overflow) {
            if( n_ > 0 ) indices_ = new int[n_];
            if( cbt_ != 0 ) valuation_ = new BeliefTracking::valuation_t(cbt_->nvars());
            advance_until_valid();
        }
        ~const_join_iterator_type1() {
            delete valuation_;
            delete[] indices_;
        }

        bool operator==(const const_join_iterator_type1 &it) const {
            return (overflow_ && it.overflow_) || (!overflow_ && !it.overflow_ && !bcmp(indices_, it.indices_, n_ * sizeof(int)));
        }
        bool operator!=(const const_join_iterator_type1 &it) const {
            return !(*this == it);
        }
        const const_join_iterator_type1& operator=(const const_join_iterator_type1 &it) {
            if( it.overflow_ ) {
                overflow_ = true;
            } else {
                if( valuation_ == 0 ) valuation_ = new BeliefTracking::valuation_t;
                probability_ = it.probability_;
                *valuation_ = *it.valuation_;
                cbt_ = it.cbt_;
                overflow_ = it.overflow_;
                n_ = it.n_;
                bcopy(it.indices_, indices_, n_ * sizeof(int));
            }
            return *this;
        }

        const const_join_iterator_type1& operator++() {
            // add one to indices (from left to right w/ overflow at each
            // column determined by domain size of corresponding variable).
            // Keep doing this until hitting a consistent valuation with
            // positive mass.
            advance_one_step();
            advance_until_valid();
            return *this;
        }
        T get_probability() const { return probability_; }
        const BeliefTracking::valuation_t& get_valuation() const { return *valuation_; }

      private:
        void print_indices(std::ostream &os) const {
            os << "indices:";
            for( int k = 0; k < n_; ++k ) os << " " << indices_[k];
            os << std::endl;
        }

        void advance_one_step() {
            int k = 0;
            ++indices_[k];
            while( (k < n_) && (indices_[k] == cbt_->beam(k).nvaluations()) ) {
                indices_[k] = 0;
                ++indices_[++k];
            }
            if( k == n_ ) overflow_ = true;
        }

        void advance_until_valid() {
            if( !overflow_ ) make_up_valuation();
            while( !overflow_ && invalid_ ) {
                advance_one_step();
                make_up_valuation();
            }
        }

        // Extend the current (partial) global valuation kept in valuation_
        // with the given partial valuation. If some inconsistency is detected,
        // return false. Otherwise, return true.
        bool extend_valuation(const beam_t<T> &beam) {
            for( int k = 0; k < beam.nvars(); ++k ) {
                int var = beam.inverse_map_variable(k);
                int value = beam.valuation()[k];
                int old_value = (*valuation_)[var];
                (*valuation_)[var] = value;
                if( (old_value != -1) && (old_value != value) ) return false;
            }
            return true;
        }

        void make_up_valuation() {
            // construct a global valuation with the partial valuations
            // indicated by indices_. If this cannot be done in a consistent
            // manner or resulting probability is zero, return and set the
            // invalid flag to true.
    
            assert(valuation_ != 0);
            for( int k = 0; k < cbt_->nvars(); ++k ) (*valuation_)[k] = -1;

            probability_ = 1;
            invalid_ = false;
            for( int k = 0; !invalid_ && (k < cbt_->nbeams()); ++k ) {
                const beam_t<T> &beam = cbt_->beam(k);
                T p = beam.probability(indices_[k]);
                if( p == 0 ) {
                    invalid_ = true;
                } else {
                    beam.decode_index(indices_[k]);
                    invalid_ = !extend_valuation(beam);
                    probability_ *= p;
                }
            }
        }
    };

    struct const_join_iterator_type2 {
        T probability_;
        BeliefTracking::valuation_t *valuation_;
        const causal_belief_tracking_t *cbt_;
        const joint_distribution_t<T> *joint_;

        int index_;
        int max_index_;

        mutable BeliefTracking::valuation_t *aux_valuation_;

        const_join_iterator_type2(const causal_belief_tracking_t *cbt = 0, int index = 0, const joint_distribution_t<T> *joint = 0)
          : probability_(0), valuation_(0), cbt_(cbt), joint_(joint), index_(index), max_index_(0), aux_valuation_(0) {
            if( cbt_ != 0 ) {
                max_index_ = cbt_->max_num_valuations();
                if( index != -1 ) {
                    valuation_ = new BeliefTracking::valuation_t(cbt_->nvars());
                    aux_valuation_ = new BeliefTracking::valuation_t(cbt_->nvars());
                } else {
                    index_ = max_index_;
                }
            }
            if( (cbt_ != 0) && (index_ < max_index_) ) {
                decode_index(index_);
                advance_until_valid();
            }
        }
        ~const_join_iterator_type2() { delete aux_valuation_; delete valuation_; }

        bool operator==(const const_join_iterator_type2 &it) const {
            return index_ == it.index_;
        }
        bool operator!=(const const_join_iterator_type2 &it) const {
            return !(*this == it);
        }
        const const_join_iterator_type2& operator=(const const_join_iterator_type2 &it) {
            probability_ = it.probability_;
            cbt_ = it.cbt_;
            index_ = it.index_;
            max_index_ = it.max_index_;
            if( it.valuation_ != 0 ) {
                if( valuation_ == 0 ) {
                    valuation_ = new BeliefTracking::valuation_t(cbt_->nvars());
                    aux_valuation_ = new BeliefTracking::valuation_t(cbt_->nvars());
                }
                *valuation_ = it.valuation_;
            } else {
                delete[] aux_valuation_;
                delete[] valuation_;
                valuation_ = 0;
                aux_valuation_ = 0;
            }
            return *this;
        }

        const const_join_iterator_type2& operator++() {
            // add one to indices (from left to right w/ overflow at each
            // column determined by domain size of corresponding variable).
            // Keep doing this until hitting a consistent valuation with
            // positive mass.
            advance_one_step();
            advance_until_valid();
            return *this;
        }

        int get_index() const { return index_; }
        T get_probability() const { return probability_; }
        const BeliefTracking::valuation_t& get_valuation() const { return *valuation_; }

      private:
        void advance_one_step() {
            ++index_;
            decode_index(index_);
        }

        void advance_until_valid() {
            assert(valuation_ != 0);
            while( (index_ < max_index_) && !is_consistent_valuation(*valuation_, *joint_) )
                advance_one_step();
        }

        void clear_valuation(BeliefTracking::valuation_t &valuation, int nvars) const {
            assert(valuation.size() >= nvars);
            for( int k = 0; k < nvars; ++k )
                valuation[k] = -1;
        }

        void decode_index(int index) const {
            assert((cbt_ != 0) && (valuation_ != 0));
            clear_valuation(*valuation_, cbt_->nvars());
            for( int var = cbt_->nvars() - 1; var >= 0; --var ) {
                (*valuation_)[var] = index % cbt_->domains()[var];
                index = index / cbt_->domains()[var];
            }
        }

        bool is_consistent_valuation(const BeliefTracking::valuation_t &valuation, const joint_distribution_t<T> &joint) const {
            assert((cbt_ != 0) && (aux_valuation_ != 0));
            //std::cout << "BEGIN: is_consistent_valuation: val=" << valuation << std::endl;
            T prob = 1;
            clear_valuation(*aux_valuation_, cbt_->nvars());
            for( int k = 0; k < cbt_->nbeams(); ++k ) {
                const beam_t<T> &beam = cbt_->beam(k);
                bool inconsistent_with_beam = true;
                for( int beam_index = 0; inconsistent_with_beam && (beam_index < beam.nvaluations()); ++beam_index ) {
                    if( beam.table()[beam_index] == 0 ) continue;
                    beam.decode_index(beam_index);
                    inconsistent_with_beam = !beam.is_consistent_with_beam_valuation(valuation);
                    if( !inconsistent_with_beam ) {
                        // we found a beam valuation that is consistent with given
                        // valuation. Update probability of valuation using the
                        // probability of separator
                        BeliefTracking::event_t separator;
                        separator.reserve(beam.nvars());
                        for( int i = 0; i < beam.nvars(); ++i ) {
                            int varid = beam.inverse_map_variable(i);
                            if( (*aux_valuation_)[varid] != -1 ) {
                                assert((*aux_valuation_)[varid] == valuation[varid]);
                                separator.push_back(std::make_pair(i, valuation[varid]));
                            }
                            (*aux_valuation_)[varid] = valuation[varid];
                        }
                        prob = prob * beam.xprobability(beam_index) / beam.probability(separator);
                        //std::cout << "Beam=" << beam.beam_string() << ", sep=" << separator << std::flush << ", p=" << beam.probability(separator) << std::endl;
                    }
                }
                if( inconsistent_with_beam ) return false;
            }
            //std::cout << "CONSISTENT: probability=" << prob << std::endl;
            //std::cout << "val=" << valuation << ", p=" << prob << std::endl;
            const_cast<T&>(probability_) = prob;
            if( prob != joint.probability(valuation) ) {
                std::cout << "XXXXXX: index=" << index_ << ", val=" << valuation << ", p=" << prob << ", joint=" << joint.probability(valuation) << std::endl;
            }
            return true;
        }
    };

  protected:
    std::vector<beam_t<T> > beams_;
    std::set<int> variables_;
    std::vector<int> domains_;
    int max_num_valuations_;

  public:
    const joint_distribution_t<T> *joint_;
    causal_belief_tracking_t(int nbeams = 0, const joint_distribution_t<T> *joint = 0) : beams_(nbeams), joint_(joint) { }
    virtual ~causal_belief_tracking_t() { }

    int nvars() const { return variables_.size(); }
    int nbeams() const { return beams_.size(); }
    const std::vector<beam_t<T> >& beams() const { return beams_; }
    const beam_t<T>& beam(int k) const { return beams_[k]; }
    const std::vector<int>& domains() const { return domains_; }
    int max_num_valuations() const { return max_num_valuations_; }

    void calculate_variables() {
        variables_.clear();
        for( int k = 0; k < beams_.size(); ++k ) {
            const std::set<int> &beam_vars = beams_[k].variables();
            variables_.insert(beam_vars.begin(), beam_vars.end());
        }

        domains_ = std::vector<int>(variables_.size(), -1);
        for( int k = 0; k < beams_.size(); ++k ) {
            const std::set<int> &beam_vars = beams_[k].variables();
            for( std::set<int>::const_iterator it = beam_vars.begin(); it != beam_vars.end(); ++it ) {
                int varid = beams_[k].map_variable(*it);
                int dsz = beams_[k].domains()[varid];
                assert((domains_[*it] == -1) || (domains_[*it] == dsz));
                domains_[*it] = dsz;
            }
        }

        max_num_valuations_ = 1;
        for( int k = 0; k < domains_.size(); ++k )
            max_num_valuations_ *= domains_[k];
    }

    static void clear(std::vector<beam_t<T> > &beams) {
        for( size_t k = 0; k < beams.size(); ++k )
            beams[k].clear();
    }
    void clear() { clear(beams_); }

    static void set_uniform_distribution(std::vector<beam_t<T> > &beams) {
        for( size_t k = 0; k < beams.size(); ++k )
            beams[k].set_uniform_distribution();
    }
    void set_uniform_distribution() { set_uniform_distribution(beams_); }

    static void normalize(std::vector<beam_t<T> > &beams) {
        for( size_t k = 0; k < beams.size(); ++k )
            beams[k].normalize();
    }
    void normalize() { normalize(beams_); }

    const_join_iterator_type2 begin_join(const joint_distribution_t<T> *joint) const {
        //return const_join_iterator_type1(this, beams_.size());
        return const_join_iterator_type2(this, 0, joint);
    }
    const_join_iterator_type2 end_join() const {
        //return const_join_iterator_type1(0, 0, true);
        return const_join_iterator_type2(this, -1);
    }

    void project(const BeliefTracking::valuation_t &valuation, float p, std::vector<beam_t<T> > &nbeams) const {
        if( p > 0 ) {
            for( int k = 0; k < beams_.size(); ++k )
                nbeams[k].project(valuation, p);
        }
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

    void my_verify_join(const joint_distribution_t<T> *joint) const {
        std::cout << "verify_joint: joint=" << joint << std::endl;
        for( const_join_iterator_type2 it = begin_join(joint); it != end_join(); ++it );
        std::cout << "verify_joint: END" << std::endl;
    }

    virtual void progress_and_filter(int action, int obs,
                                     BeliefTracking::belief_tracking_t::det_progress_func_t progress,
                                     BeliefTracking::belief_tracking_t::filter_func_t filter)
    {
        my_verify_join(joint_);

        BeliefTracking::valuation_t valuation(variables_.size(), 0);
        std::vector<beam_t<T> > nbeams(beams_);
        clear(nbeams);
        std::cout << "about to iterate over join" << std::endl;
        //int n = 0;
        for( const_join_iterator_type2 it = begin_join(joint_); it != end_join(); ++it ) {
            //++n;
            std::cout << "valid valuation in join: index=" << it.get_index() << ", val=" << it.get_valuation() << std::endl;
            assert(it.get_probability() > 0);
            valuation = it.get_valuation();
            (this->*progress)(action, valuation);
            float p_obs = (this->*filter)(obs, action, valuation);
            project(valuation, it.get_probability() * p_obs, nbeams);
        }
        normalize(nbeams);
        //std::cout << "done with join: n=" << n << std::endl;
        //print(std::cout, nbeams);
        beams_ = nbeams;
    }

    static void print(std::ostream &os, const std::vector<beam_t<T> > &beams) {
        for( int k = 0; k < beams.size(); ++k )
            beams[k].print(os);
    }

    virtual void print(std::ostream &os) const {
        print(os, beams_);
    }

};

}; // end of namespace ExactDistribution

#undef DEBUG

#endif

