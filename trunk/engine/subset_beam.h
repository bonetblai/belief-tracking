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


#ifndef SUBSET_BEAM_H
#define SUBSET_BEAM_H

#include "ordered_vector.h"
#include <cassert>
#include <iostream>
#include <vector>

class subset_beam_t {
    ordered_vector_t beam_;

    static int n_;
    static int k_;
    static int max_particle_;
    static int *comb_;

    static int& comb_ref(int i, int j) { return comb_[i * (k_ + 1) + j]; }

  public:
    subset_beam_t() { }
    explicit subset_beam_t(const subset_beam_t &beam) : beam_(beam.beam_) { }
    subset_beam_t(subset_beam_t &&beam) : beam_(std::move(beam.beam_)) { }
    ~subset_beam_t() { }

    static void initialize(int n, int k) {
        n_ = n;
        k_ = k;

        // allocate and fill table of combinatorics
        comb_ = new int[(n_ + 1) * (k_ + 1)];
        for( int i = 0; i <= n_; ++i ) {
            for( int j = 0; j <= k_; ++j ) {
                if( i < j ) {
                    comb_ref(i, j) = 0;
                } else if( (j == 0) || (j == i) ) {
                    comb_ref(i, j) = 1;
                } else {
                    comb_ref(i, j) = comb_ref(i-1, j) + comb_ref(i-1, j-1);
                }
                //std::cout << "comb(" << i << "," << j << ")=" << comb_ref(i, j) << std::endl;
            }
        }
        max_particle_ = comb_[n_ * (k_ + 1) + k_];

        std::cout << "subset_beam_t::initialize:"
                  << " n=" << n
                  << ", k=" << k
                  << ", max-particle=" << max_particle_
                  << std::endl;
    }

    static int comb(int i, int j) { return comb_[i * (k_ + 1) + j]; }

    // combinatorial number system (a.k.a. combinadics)
    static void get_subset(int particle, int *subset) {
        //std::cout << "decode: p=" << particle << std::endl;
        for( int k = k_; k > 0; --k ) {
            int i = k - 1;
            for( ; (i < n_) && (comb(1 + i, k) <= particle); ++i);
            assert(comb(i, k) <= particle);
            assert(comb(1+i, k) > particle);
            subset[k_ - k] = i;
            particle -= comb(i, k);
            assert((k == k_) || (subset[k_ - k] < subset[k_ - k - 1]));
            //std::cout << "subset[" << k_ - k << "]=" << subset[k_ - k] << std::endl;
        }
    }
    static int get_particle(const int *subset) {
        int particle = 0;
        for( int k = 0; k < k_; ++k ) {
            assert((k == 0) || (subset[k] < subset[k - 1]));
            int c = subset[k] < k_ - k ? 0 : comb(subset[k], k_ - k);
            particle += c;
        }
        return particle;
    }

    static int n() { return n_; }
    static int k() { return k_; }
    static int max_particle() { return max_particle_; }

    bool empty() const { return beam_.empty(); }
    int size() const { return beam_.size(); }
    bool known() const { return beam_.size() == 1; }
    bool contains(int particle) const { return beam_.contains(particle); }
    bool consistent() const { return !beam_.empty(); }

    void clear() { beam_.clear(); }
    void insert(int particle) { beam_.insert(particle); }
    void erase(int particle) { beam_.erase(particle); }
    void erase_ordered_indices(const std::vector<int> &indices) {
        beam_.erase_ordered_indices(indices);
    }

    void set_initial_configuration(const std::vector<int> &known_elements) {
        beam_.clear();
        if( known_elements.empty() ) beam_.reserve(max_particle_);
        int *subset = new int[k_];
        for( int particle = 0; particle < max_particle_; ++particle ) {
            if( known_elements.empty() ) {
                beam_.push_back(particle);
            } else {
                subset_beam_t::get_subset(particle, subset);
                bool good_particle = true;
                for( int i = 0; good_particle && (i < (int)known_elements.size()); ++i ) {
                    int e = known_elements[i];
                    good_particle = false;
                    for( int j = 0; !good_particle && (j < k_); ++j ) {
                        good_particle = subset[j] == e;
                    }
                }
                if( good_particle ) beam_.push_back(particle);
            }
        }
        delete[] subset;
    }

    bool operator==(const subset_beam_t &beam) const {
        return (n_ == beam.n_) && (k_ == beam.k_) && (beam_ == beam.beam_);
    }
    bool operator!=(const subset_beam_t &beam) const {
        return *this == beam ? false : true;
    }

    const subset_beam_t& operator=(const subset_beam_t &beam) {
        n_ = beam.n_;
        k_ = beam.k_;
        max_particle_ = beam.max_particle_;
        beam_ = beam.beam_;
        return *this;
    }

    void print(std::ostream &os) const {
        os << "{";
#if 0 // print function
        for( const_iterator it = begin(); it != end(); ++it ) {
            int particle = *it;
            os << "[p=" << particle << ":";
            for( int var = 0; var < nvars_; ++var ) {
                int val = value(var, particle);
                os << "v" << var << "=" << val << ",";
            }
            os << "],";
        }
#endif
        os << "},sz=" << size() << ",cap=" << beam_.capacity();
    }

    typedef ordered_vector_t::const_iterator const_iterator;
    const_iterator begin() const { return beam_.begin(); }
    const_iterator end() const { return beam_.end(); }

    void filter_has_element(int element) {
        assert(0); // error: not yet implemented
    }
};

int subset_beam_t::n_ = 0;
int subset_beam_t::k_= 0;
int subset_beam_t::max_particle_= 0;
int *subset_beam_t::comb_= 0;

inline std::ostream& operator<<(std::ostream &os, const subset_beam_t &beam) {
    beam.print(os);
    return os;
}

#endif

