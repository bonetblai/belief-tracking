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

#ifndef VAR_BEAM_H
#define VAR_BEAM_H

#include "ordered_vector.h"
#include <cassert>
#include <iostream>
#include <vector>

//#define DEBUG

// This beam type contains a number of different variables, each with
// its own domain size. 
class var_beam_t {
    int nvars_;
    int *domsz_;
    int max_value_;
    int initial_size_;
    ordered_vector_t beam_;

  public:
    var_beam_t(int nvars = 1, int domsz = 0)
      : nvars_(nvars), domsz_(new int[nvars_]), max_value_(0) {
        for( int var = 0; var < nvars_; ++var )
            domsz_[var] = domsz;
        calculate_max_value();
        initial_size_ = 0;
#ifdef DEBUG
        std::cout << "var_beam_t: nvars=" << nvars_ << ", max-value=" << max_value_ << std::endl;
#endif
    }
    explicit var_beam_t(const var_beam_t &beam)
      : nvars_(beam.nvars_), domsz_(new int[nvars_]), max_value_(beam.max_value_) {
        memcpy(domsz_, beam.domsz_, nvars_ * sizeof(int));
        beam_ = beam.beam_;
        initial_size_ = beam.initial_size_;
#ifdef DEBUG
        std::cout << "var_beam_t: copy-constructor: nvars=" << nvars_ << ", max-value=" << max_value_ << std::endl;
#endif
    }
    var_beam_t(var_beam_t &&beam)
      : nvars_(beam.nvars_), domsz_(beam.domsz_),
        max_value_(beam.max_value_), initial_size_(beam.initial_size_),
        beam_(std::move(beam.beam_)) {
        beam.domsz_ = 0;
#ifdef DEBUG
        std::cout << "var_beam_t: move-constructor: nvars=" << nvars_ << ", max-value=" << max_value_ << std::endl;
#endif
    }
    ~var_beam_t() { delete[] domsz_; }

    uint32_t hash() const { return beam_.hash(); }
    int nvars() const { return nvars_; }
    int domsz(int var) const { return domsz_[var]; }
    int max_value() const { return max_value_; }
    void set_domsz(int var, int domsz) {
        domsz_[var] = domsz;
    }

    void set_initial_size(int initial_size) { initial_size_ = initial_size; }
    int initial_size() const { return initial_size_; }

    void calculate_max_value() {
        max_value_ = 1;
        for( int var = 0; var < nvars_; ++var )
            max_value_ *= domsz_[var];
    }

    int value(int var, int valuation) const {
        int i = 0;
        for( ; var > 0; --var, valuation /= domsz_[i++]);
        return valuation % domsz_[i];
    }

    bool consistent(int valuation) {
        int last_val = -1;
        for( int var = 0; var < nvars_; ++var ) {
            int val = value(var, valuation);
            if( val <= last_val ) return false;
            last_val = val;
        }
        return true;
    }

    bool empty() const { return beam_.empty(); }
    int size() const { return beam_.size(); }
    bool known() const { return beam_.size() == 1; }
    bool contains(int value) const { return beam_.contains(value); }
    bool consistent() const { return !beam_.empty(); }

    void reserve(int capacity) { beam_.reserve(capacity); }
    void clear() { beam_.clear(); }

    void insert(int e) { beam_.insert(e); }
    bool push_back(int e) {
        if( beam_.empty() || (beam_[beam_.size() - 1] < e) ) {
            beam_.push_back(e);
            return true;
        } else {
            return false;
        }
    }

    void erase(int e) { beam_.erase(e); }
    void erase_ordered_indices(const std::vector<int> &indices) {
        beam_.erase_ordered_indices(indices);
    }

    void set_initial_configuration(int value = -1) {
        beam_.clear();
        if( value == -1 ) {
            beam_.reserve(max_value_);
            for( int value = 0; value < max_value_; ++value ) {
                if( consistent(value) ) {
                    beam_.insert(value);
                }
            }
        } else {
            beam_.insert(value);
        }
#ifdef DEBUG
        std::cout << "var_beam_t: initial_configuration: sz=" << beam_.size() << std::endl;
#endif
    }

    int operator[](int i) const { return beam_[i]; }

    bool operator==(const var_beam_t &beam) const {
        return beam_ == beam.beam_;
    }
    bool operator!=(const var_beam_t &beam) const {
        return *this == beam ? false : true;
    }

    const var_beam_t& operator=(const var_beam_t &beam) {
        if( nvars_ != beam.nvars_ ) {
            delete[] domsz_;
            domsz_ = new int[beam.nvars_];
            nvars_ = beam.nvars_;
        }
        memcpy(domsz_, beam.domsz_, nvars_ * sizeof(int));
        max_value_ = beam.max_value_;
        initial_size_ = beam.initial_size_;
        beam_ = beam.beam_;
        return *this;
    }

    void print(std::ostream &os) const {
        os << "{";
        for( const_iterator it = begin(); it != end(); ++it ) {
            int valuation = *it;
#if 1
            os << "[p=" << valuation << ":";
            for( int var = 0; var < nvars_; ++var ) {
                int val = value(var, valuation);
                os << "v" << var << "=" << val << ",";
            }
            os << "],";
#else
            os << valuation << ",";
#endif
        }
        os << "},sz=" << size() << ",cap=" << beam_.capacity();
    }

    typedef ordered_vector_t::const_iterator const_iterator;
    const_iterator begin() const { return beam_.begin(); }
    const_iterator end() const { return beam_.end(); }

    void filter(int element, bool different) {
        static std::vector<int> indices_to_erase;
        indices_to_erase.clear();
        indices_to_erase.reserve(beam_.size());
        for( const_iterator it = begin(); it != end(); ++it ) {
            if( (different && (element != *it)) || (!different && (element == *it)) )
                indices_to_erase.push_back(it.index());
        }
        beam_.erase_ordered_indices(indices_to_erase);
    }
};

inline std::ostream& operator<<(std::ostream &os, const var_beam_t &beam) {
    beam.print(os);
    return os;
}

#undef DEBUG

#endif

