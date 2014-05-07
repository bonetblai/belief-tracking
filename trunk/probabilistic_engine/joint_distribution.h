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

#ifndef JOINT_DISTRIBUTION_H
#define JOINT_DISTRIBUTION_H

#include <iostream>
#include <iomanip>
#include <cassert>
#include <vector>

//#define DEBUG

namespace ProbabilisticBeliefTracking {

template<typename T> class joint_distribution_t {
  protected:
    int nvars_;
    int nvaluations_;
    int non_zero_entries_;
    T normalizer_;

    std::vector<int> domains_;
    T *table_;

    mutable BeliefTracking::valuation_t valuation_;

  public:
    joint_distribution_t() : nvars_(0), nvaluations_(0), normalizer_(1), table_(0) { }
    joint_distribution_t(int nvars, const std::vector<int> &domains) : table_(0) {
        normalizer_ = 1;
        allocate(nvars, domains);
    }
    joint_distribution_t(int nvars, int domain) : table_(0) {
        normalizer_ = 1;
        std::vector<int> domains(nvars, domain);
        allocate(nvars, domains);
    }
    virtual ~joint_distribution_t() { delete[] table_; }

    void allocate(int nvars, int domain) {
        std::vector<int> domains(nvars, domain);
        allocate(nvars, domains);
    }
    void allocate(int nvars, const std::vector<int> &domains) {
        nvars_ = nvars;
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
    float probability(const BeliefTracking::event_t &event) const {
        T mass = 0;
        for( int k = 0; k < nvaluations_; ++k ) {
            if( is_event_instance(k, event) ) mass += table_[k];
        }
        return mass / normalizer_;
    }

    T& operator[](int index) { return probability(index); }
    T operator[](int index) const { return probability(index); }
    T operator[](const BeliefTracking::event_t &event) const { return probability(event); }

    void print(std::ostream &os, int prec, bool decode) const {
        int old_prec = os.precision();
        os << std::setprecision(prec)
           << "distribution-table:" << std::endl;
        for( int k = 0; k < nvaluations_; ++k ) {
            if( table_[k] != 0 ) {
                os << "  " << k;
                if( decode ) {
                    decode_index(k);
                    os << ":" << valuation_;
                }
                os << "=" << table_[k] << std::endl;
            }
        }
        os << "normalizer=" << normalizer_ << std::endl
           << "#non-zero-entries=" << non_zero_entries_ << std::endl
           << std::setprecision(old_prec);
    }
};

}; // end of namespace ProbabilisticBeliefTracking

#undef DEBUG

#endif

