/*
 *  Copyright (C) 2016 Universidad Simon Bolivar
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

#ifndef WEIGHTED_VAR_BEAM_H
#define WEIGHTED_VAR_BEAM_H

#include "weighted_ordered_vector.h"
#include <cassert>
#include <iostream>
#include <vector>

//#define DEBUG

// This beam type contains a number of different variables, each with
// its own domain size. 
class weighted_var_beam_t {
    int nvars_;
    int *domsz_;
    int max_value_;
    int initial_size_;
    weighted_ordered_vector_t beam_;

  public:
    weighted_var_beam_t(int nvars = 1, int domsz = 0)
      : nvars_(nvars), domsz_(new int[nvars_]), max_value_(0) {
        for( int var = 0; var < nvars_; ++var )
            domsz_[var] = domsz;
        calculate_max_value();
        initial_size_ = 0;
#ifdef DEBUG
        std::cout << "weighted_var_beam_t: nvars=" << nvars_ << ", max-value=" << max_value_ << std::endl;
#endif
    }
    explicit weighted_var_beam_t(const weighted_var_beam_t &beam)
      : nvars_(beam.nvars_), domsz_(new int[nvars_]), max_value_(beam.max_value_) {
        memcpy(domsz_, beam.domsz_, nvars_ * sizeof(int));
        beam_ = beam.beam_;
        initial_size_ = beam.initial_size_;
#ifdef DEBUG
        std::cout << "weighted_var_beam_t: copy-constructor: nvars=" << nvars_ << ", max-value=" << max_value_ << std::endl;
#endif
    }
    weighted_var_beam_t(weighted_var_beam_t &&beam)
      : nvars_(beam.nvars_), domsz_(beam.domsz_),
        max_value_(beam.max_value_), initial_size_(beam.initial_size_),
        beam_(std::move(beam.beam_)) {
        beam.domsz_ = 0;
#ifdef DEBUG
        std::cout << "weighted_var_beam_t: move-constructor: nvars=" << nvars_ << ", max-value=" << max_value_ << std::endl;
#endif
    }
    ~weighted_var_beam_t() { delete[] domsz_; }

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

    int min_weight() const { return beam_.min_weight(); }
    int max_weight() const { return beam_.max_weight(); }

    void normalize() {
        int alpha = min_weight();
        if( alpha > 0 ) beam_.substract(alpha);
    }
    bool normalized() {
        return min_weight() == 0;
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
    int weight(int value) const { return beam_.weight(value); }
    bool consistent() const { return !beam_.empty(); }

    int value_by_index(int index) const { return beam_.value_by_index(index); }
    int weight_by_index(int index) const { return beam_.weight_by_index(index); }

    void reserve(int capacity) { beam_.reserve(capacity); }
    void clear() { beam_.clear(); }

    void increase_weight(int index, int amount, int cap = std::numeric_limits<int>::max()) {
        beam_.increase_weight(index, amount, cap);
    }
    void increase_weight(const std::pair<int, int> &p, int cap = std::numeric_limits<int>::max()) {
        beam_.increase_weight(p.first, p.second, cap);
    }
    void set_weight(int index, int w) {
        beam_.set_weight(index, w);
    }

    void insert(int e, int w) { beam_.insert(e, w); }
    bool push_back(int e, int w) {
        if( beam_.empty() || (beam_[beam_.size() - 1].first < e) ) {
            beam_.push_back(e, w);
            return true;
        } else {
            return false;
        }
    }

    void erase(int e) { beam_.erase(e); }
    void erase_ordered_indices(const std::vector<int> &indices) {
        beam_.erase_ordered_indices(indices);
    }

    void set_initial_configuration(int value = -1, int weight = 0) {
        beam_.clear();
        if( value == -1 ) {
            beam_.reserve(max_value_);
            for( int value = 0; value < max_value_; ++value ) {
                if( consistent(value) ) {
                    beam_.insert(value, weight);
                }
            }
        } else {
            beam_.insert(value, weight);
        }
#ifdef DEBUG
        std::cout << "weighted_var_beam_t: initial_configuration: sz=" << beam_.size() << std::endl;
#endif
    }

    std::pair<int, int> operator[](int i) const { return beam_[i]; }

    bool operator==(const weighted_var_beam_t &beam) const {
        return beam_ == beam.beam_;
    }
    bool operator!=(const weighted_var_beam_t &beam) const {
        return *this == beam ? false : true;
    }

    const weighted_var_beam_t& operator=(const weighted_var_beam_t &beam) {
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
            os << "[v=" << valuation << ":";
            for( int var = 0; var < nvars_; ++var ) {
                int val = value(var, valuation);
                os << "x" << var << "=" << val;
                if( var + 1 < nvars_ ) os << ",";
            }
            os << ":w=" << it.weight() << "],";
#else
            os << valuation << ",";
#endif
        }
        os << "},sz=" << size() << ",cap=" << beam_.capacity();
    }

    typedef weighted_ordered_vector_t::const_iterator const_iterator;
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

inline std::ostream& operator<<(std::ostream &os, const weighted_var_beam_t &beam) {
    beam.print(os);
    return os;
}

#undef DEBUG

#endif

