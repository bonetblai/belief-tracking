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

#ifndef FULL_JOINT_TRACKING_H
#define FULL_JOINT_TRACKING_H

#include <iostream>
#include <iomanip>
#include <cassert>
#include <vector>
#include <strings.h>

#include "probabilistic_belief_tracking.h"
#include "joint_distribution.h"

//#define DEBUG

namespace ProbabilisticBeliefTracking {

template<typename T> class full_joint_tracking_t : public joint_distribution_t<T>, public probabilistic_belief_tracking_t {
  protected:
    using joint_distribution_t<T>::nvaluations_;
    using joint_distribution_t<T>::valuation_;
    using joint_distribution_t<T>::table_;

  public:
    full_joint_tracking_t() { }
    full_joint_tracking_t(int nvars, int domain) : joint_distribution_t<T>(nvars, domain) { }
    virtual ~full_joint_tracking_t() { }

    virtual float probability(const BeliefTracking::event_t &event) const {
        return joint_distribution_t<T>::probability(event);
    }

    virtual void progress_and_filter(int action, int obs,
                                     BeliefTracking::belief_tracking_t::det_progress_func_t progress,
                                     BeliefTracking::belief_tracking_t::filter_func_t filter) {
        T *ntable = new T[nvaluations_];
        bzero(ntable, sizeof(T) * nvaluations_);
        for( int k = 0; k < nvaluations_; ++k ) {
            if( table_[k] > 0 ) {
                joint_distribution_t<T>::decode_index(k);
                (this->*progress)(action, valuation_);
                float p = (this->*filter)(obs, action, valuation_);
                int i = joint_distribution_t<T>::encode_valuation(valuation_);
                assert(i == k);
                assert(p == 0 || p == 1);
                ntable[i] = table_[k] * p;
                assert(ntable[i] == table_[k] || ntable[i] == 0);
            }
        }
        delete[] table_;
        table_ = ntable;
        joint_distribution_t<T>::normalize();
    }

    virtual void print(std::ostream &os) const { joint_distribution_t<T>::print(os, 2, false); }
};


#if 0
template<typename T> class xfull_joint_tracking_t : public probabilistic_belief_tracking_t {
  protected:
    int nvars_;
    int nvaluations_;
    int non_zero_entries_;
    T normalizer_;

    std::vector<int> variables_;
    std::vector<int> domains_;
    T *table_;

    mutable BeliefTracking::valuation_t valuation_;

  public:
    xfull_joint_tracking_t() : nvars_(0), nvaluations_(0), normalizer_(1), table_(0) { }
    xfull_joint_tracking_t(int nvars, const std::vector<int> &domains) : table_(0) {
        normalizer_ = 1;
        allocate(nvars, domains);
    }
    xfull_joint_tracking_t(int nvars, int domain) : table_(0) {
        normalizer_ = 1;
        std::vector<int> domains(nvars, domain);
        allocate(nvars, domains);
    }
    virtual ~xfull_joint_tracking_t() { delete[] table_; }

    void allocate(int nvars, const std::vector<int> &domains) {
        nvars_ = nvars;
        variables_ = std::vector<int>(nvars_, 0);
        for( int i = 0; i < nvars_; ++i ) variables_[i] = i;
        domains_ = domains;
        assert(domains_.size() >= nvars_);
        while( domains_.size() > nvars_ ) domains_.pop_back();
        nvaluations_ = 1;
        for( int i = 0; i < nvars_; ++i ) nvaluations_ *= domains_[i];
        delete[] table_;
        table_ = new T[nvaluations_];
        bzero(table_, sizeof(T) * nvaluations_);
        non_zero_entries_ = 0;
        valuation_ = BeliefTracking::valuation_t(nvars_);
    }

    void set_variables(const std::vector<int> &variables) {
        variables_ = variables;
    }

    void clear() {
        bzero(table_, nvaluations_ * sizeof(T));
        non_zero_entries_ = 0;
        normalizer_ = 1;
    }

    int nvars() const { return nvars_; }
    int nvaluations() const { return nvaluations_; }
    int non_zero_entries() const { return non_zero_entries_; }
    T normalizer() const { return normalizer_; }
    const T* table() const { return table_; }
    const BeliefTracking::valuation_t& valuation() const { return valuation_; }

    void normalize() {
        normalizer_ = 0;
        non_zero_entries_ = 0;
        for( int k = 0; k < nvaluations_; ++k ) {
            normalizer_ += table_[k];
            if( table_[k] > 0 ) ++non_zero_entries_;
        }
    }

    void set_uniform_distribution() {
        normalizer_ = nvaluations_;
        for( int k = 0; k < nvaluations_; ++k )
            table_[k] = 1;
        non_zero_entries_ = nvaluations_;
    }

    void decode_index(int index, BeliefTracking::valuation_t &valuation) const {
        bzero(&valuation[0], nvars_ * sizeof(int));
        for( int var = nvars_ - 1; var >= 0; --var ) {
            valuation[var] = index % domains_[var];
            index = index / domains_[var];
        }
        assert(index == 0);
    }
    void decode_index(int index) const { decode_index(index, valuation_); }
    int encode_valuation(const BeliefTracking::valuation_t &valuation) {
        int index = 0;
        for( int var = 0; var < nvars_; ++var )
            index = index * domains_[var] + valuation[var];
        return index;
    }

    bool is_event_instance(int index, const BeliefTracking::event_t &event) const {
        decode_index(index);
        for( size_t k = 0; k < event.size(); ++k ) {
            if( valuation_[event[k].first] != event[k].second )
                return false;
        }
        return true;
    }

    T& probability(int index) { return table_[index]; }
    T probability(int index) const { return table_[index] / normalizer_; }
    virtual float probability(const BeliefTracking::event_t &event) const {
        T mass = 0;
        for( int k = 0; k < nvaluations_; ++k ) {
            if( is_event_instance(k, event) ) mass += table_[k];
        }
        return mass / normalizer_;
    }

    T& operator[](int index) { return probability(index); }
    T operator[](int index) const { return probability(index); }

    void print_valuation(std::ostream &os, const BeliefTracking::valuation_t &valuation) const {
        os << "[";
        for( int var = 0; var < nvars_; ++var ) {
            os << valuation[var];
            if( var < nvars_ - 1 ) os << ",";
        }
        os << "]";
    }
    void print_valuation(std::ostream &os) const { print_valuation(os, valuation_); }

    void print(std::ostream &os, int prec, bool decode) const {
        int old_prec = os.precision();
        os << std::setprecision(prec)
           << "distribution-table:" << std::endl;
        for( int k = 0; k < nvaluations_; ++k ) {
            if( table_[k] != 0 ) {
                os << "  " << k;
                if( decode ) {
                    decode_index(k);
                    os << ":";
                    print_valuation(os);
                }
                os << "=" << table_[k] << std::endl;
            }
        }
        os << "normalizer=" << normalizer_ << std::endl
           << "#non-zero-entries=" << non_zero_entries_ << std::endl
           << std::setprecision(old_prec);
    }

    virtual void progress_and_filter(int action, int obs,
                                     BeliefTracking::belief_tracking_t::det_progress_func_t progress,
                                     BeliefTracking::belief_tracking_t::filter_func_t filter) {
        T *ntable = new T[nvaluations_];
        bzero(ntable, sizeof(T) * nvaluations_);
        for( int k = 0; k < nvaluations_; ++k ) {
            if( table_[k] > 0 ) {
                decode_index(k);
                (this->*progress)(action, valuation_);
                float p = (this->*filter)(obs, action, valuation_);
                int i = encode_valuation(valuation_);
                ntable[i] = table_[k] * p;
            }
        }
        delete[] table_;
        table_ = ntable;
        normalize();
    }

    virtual void print(std::ostream &os) const { print(os, 2, false); }

};
#endif

}; // end of namespace ProbabilisticBeliefTracking

#undef DEBUG

#endif

