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


#ifndef DISTRIBUTION_H
#define DISTRIBUTION_H

#include <cassert>
#include <iostream>
#include <vector>
#include <string.h>

//#define DEBUG

class distribution_t {
    float *vector_;
    int size_;

  public:
    distribution_t(int size) : size_(size) {
        vector_ = new float[size_];
    }
    explicit distribution_t(const distribution_t &dist) : size_(dist.size_) {
        vector_ = new float[size_];
        *this = dist;
    }
    distribution_t(distribution_t &&dist)
      : vector_(dist.vector_), size_(dist.size_) {
        dist.vector_ = 0;
        vec.size_ = 0;
    }
    ~distribution_t() { delete[] vector_; }

    void clear() { memset(vector_, 0, size_ * sizeof(float)); }
    int size() const { return size_; }

    float operator[](int i) const { return vector_[i]; }
    float& operator[](int i) { return vector_[i]; }

    const distribution_t& operator=(const distribution_t &dist) {
        assert(size_ == dist.size_);
        memcpy(vector_, dist.vector_, dist.size_ * sizeof(float));
        return *this;
    }
    const distribution_t& operator=(distribution_t &&dist) {
        delete[] vector_;
        vector_ = dist.vector_;
        size_ = dist.size_;
        dist.vector_ = 0;
        dist.size_ = 0;
        return *this;
    }

    bool operator==(const distribution_t &dist) const {
        return (size_ == dist.size_) &&
               (memcmp(vector_, dist.vector_, size_ * sizeof(float)) == 0);
    }
    bool operator!=(const distribution_t &dist) const {
        return *this == dist ? false : true;
    }

    void print(std::ostream &os) const {
        os << size_ << ":[";
        for( int i = 0; i < size_ - 1; ++i ) {
            os << vector_[i] << ',';
        }
        if( size_ > 0 ) os << vector_[size_ - 1];
        os << ']';
    }
};

inline std::ostream& operator<<(std::ostream &os, const distribution_t &dist) {
    dist.print(os);
    return os;
}

#undef DEBUG

#endif

