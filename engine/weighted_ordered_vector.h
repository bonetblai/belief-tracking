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


#ifndef WEIGHTED_ORDERED_VECTOR_H
#define WEIGHTED_ORDERED_VECTOR_H

#include <cassert>
#include <iostream>
#include <limits>
#include <vector>
#include <string.h>

#include "ordered_vector.h"

//#define DEBUG

class weighted_ordered_vector_t {
  protected:
    int *vector_;
    int *weight_;
    int capacity_;
    int size_;

    void double_reserve(int new_capacity) {
        int capacity = capacity_;
        while( capacity < new_capacity )
            capacity = capacity == 0 ? 1 : capacity << 1;
        reserve(capacity);
    }

    int binary_search(int e) const {
        if( size_ == 0 ) return -1;
        int lb = 0, ub = size_ - 1, mid = (lb + ub) / 2;
        if( e < vector_[lb] ) {
            mid = lb - 1;
        } else if( vector_[ub] <= e ) {
            mid = ub;
        } else {
            assert(vector_[lb] <= e);
            assert(e < vector_[ub]);
            while( lb != mid ) {
                if( vector_[mid] <= e ) {
                    lb = mid;
                } else {
                    ub = mid;
                }
                mid = (lb + ub) / 2;
            }
            assert(vector_[mid] <= e);
            assert(e < vector_[mid+1]);
        }
        assert((mid == -1) || (vector_[mid] <= e));
        return mid;
    }

  public:
    weighted_ordered_vector_t()
      : vector_(0), weight_(0), capacity_(0), size_(0) {
    }
    explicit weighted_ordered_vector_t(const weighted_ordered_vector_t &vec)
      : vector_(0), weight_(0), capacity_(0), size_(0) {
        *this = vec;
    }
    weighted_ordered_vector_t(weighted_ordered_vector_t &&vec)
      : vector_(vec.vector_), weight_(vec.weight_), capacity_(vec.capacity_), size_(vec.size_) {
        vec.vector_ = 0;
        vec.weight_ = 0;
        vec.capacity_ = 0;
        vec.size_ = 0;
    }
    ~weighted_ordered_vector_t() {
        delete[] vector_;
        delete[] weight_;
    }

    void reserve(int new_capacity) {
        if( capacity_ < new_capacity ) {
#ifdef DEBUG
            std::cout << "weighted_ordered_vector_t: reserve:"
                      << " old-capacity=" << capacity_
                      << ", new-capacity=" << new_capacity
                      << std::endl;
#endif
            capacity_ = new_capacity;
            int *nvector = new int[capacity_];
            memcpy(nvector, vector_, size_ * sizeof(int));
            delete[] vector_;
            vector_ = nvector;
            int *nweight = new int[capacity_];
            memcpy(nweight, weight_, size_ * sizeof(int));
            delete[] weight_;
            weight_ = nweight;
        }
    }

    uint32_t hash() const {
        uint32_t value1 = jenkins_one_at_a_time_hash(vector_, size_);
        uint32_t value2 = jenkins_one_at_a_time_hash(weight_, size_);
        return value1 ^ value2;
    }

    void clear() { size_ = 0; }
    bool empty() const { return size_ == 0; }
    int size() const { return size_; }
    int capacity() const { return capacity_; }

    int min_weight() const {
        int min = std::numeric_limits<int>::max();
        for( int i = 0; i < size_; ++i )
            min = std::min(min, weight_[i]);
        return min;
    }
    int max_weight() const {
        int max = std::numeric_limits<int>::min();
        for( int i = 0; i < size_; ++i )
            max = std::max(max, weight_[i]);
        return max;
    }

    bool contains(int e) const {
        int m = binary_search(e);
        return (m != -1) && (vector_[m] == e);
    }

    void insert(int e, int w = 1) {
        if( size_ == 0 ) {
            reserve(1);
            vector_[0] = e;
            weight_[0] = w;
            ++size_;
        } else {
            int m = binary_search(e);
            if( (m == -1) || (vector_[m] != e) ) {
                double_reserve(1 + size_);
                for( int i = size_ - 1; i > m; --i ) {
                    vector_[1+i] = vector_[i];
                    weight_[1+i] = weight_[i];
                }
                vector_[m+1] = e;
                weight_[m+1] = w;
                ++size_;
            }
        }
    }

    void push_back(int e, int w = 1) {
        if( size_ == 0 ) {
            insert(e, w);
        } else if( e > vector_[size_-1] ) {
            double_reserve(1 + size_);
            vector_[size_] = e;
            weight_[size_] = w;
            ++size_;
        } else {
            insert(e, w);
        }
    }

    // TODO: make this more efficient
    void insert(const weighted_ordered_vector_t &vec) {
        for( const_iterator it = vec.begin(); it != vec.end(); ++it ) {
            if( (size_ == 0) || (*it > vector_[size_-1]) ) {
                // copy the whole vector at the end
                int copysz = vec.size() - it.index();
                double_reserve(size_ + copysz);
                memcpy(&vector_[size_], &vec.vector_[it.index()], copysz * sizeof(int));
                memcpy(&weight_[size_], &vec.weight_[it.index()], copysz * sizeof(int));
                size_ += copysz;
                return;
            } else {
                int w = weight_[it.index()];
                insert(*it, w);
            }
        }
    }

    void erase(int e) {
        int m = binary_search(e);
        if( (m != -1) && (vector_[m] == e) ) {
            for( int i = m; i < size_ - 1; ++i ) {
                vector_[i] = vector_[i+1];
                weight_[i] = weight_[i+1];
            }
            --size_;
        }
    }

    void erase_ordered_indices(const std::vector<int> &indices) {
        int indices_sz = indices.size();
        if( indices_sz != 0 ) {
            int k = indices[0];
            for( int i = 0, j = k; j < size_; ) {
                while( (i < indices_sz) && (j == indices[i]) ) { ++j; ++i; }
                while( (j < size_) && ((i >= indices_sz) || (j < indices[i])) ) {
                    vector_[k++] = vector_[j++];
                    weight_[k++] = weight_[j++];
                }
                assert((j == size_) || (i < indices_sz));
                assert((j == size_) || (j == indices[i]));
            }
            assert(k == size_ - indices_sz);
            size_ = k;
        }
    }

    void increase_weight(int index, int amount = 1) {
        assert(index < size_);
        if( weight_[index] != std::numeric_limits<int>::max() )
            weight_[index] += amount;
    }
    void increase_weight(const std::pair<int, int> &p) {
        increase_weight(p.first, p.second);
    }
    void set_weight(int index, int weight) {
        assert(index < size_);
        weight_[index] = weight;
    }

    std::pair<int, int> operator[](int i) const {
        return std::make_pair(vector_[i], weight_[i]);
    }

    const weighted_ordered_vector_t& operator=(const weighted_ordered_vector_t &vec) {
        reserve(vec.size_);
        memcpy(vector_, vec.vector_, vec.size_ * sizeof(int));
        memcpy(weight_, vec.weight_, vec.size_ * sizeof(int));
        size_ = vec.size_;
        return *this;
    }
    const weighted_ordered_vector_t& operator=(weighted_ordered_vector_t &&vec) {
        delete[] vector_;
        delete[] weight_;
        vector_ = vec.vector_;
        weight_ = vec.weight_;
        capacity_ = vec.capacity_;
        size_ = vec.size_;
        vec.vector_ = 0;
        vec.weight_ = 0;
        vec.capacity_ = 0;
        vec.size_ = 0;
        return *this;
    }

    bool operator==(const weighted_ordered_vector_t &vec) const {
        return (size_ == vec.size_) &&
               (memcmp(vector_, vec.vector_, size_ * sizeof(int)) == 0) &&
               (memcmp(weight_, vec.weight_, size_ * sizeof(int)) == 0);
    }
    bool operator!=(const weighted_ordered_vector_t &vec) const {
        return *this == vec ? false : true;
    }

    struct const_iterator {
        const int *base_;
        const int *ptr_;
        int *wptr_;
        const_iterator(const int *ptr, int *wptr) : base_(ptr), ptr_(ptr), wptr_(wptr) { }
        ~const_iterator() { }
        const_iterator& operator++() {
            ++ptr_;
            ++wptr_;
            return *this;
        }
        int operator*() const { return *ptr_; }
        bool operator==(const const_iterator &it) const {
            return (ptr_ == it.ptr_) && (wptr_ == it.wptr_);
        }
        bool operator!=(const const_iterator &it) const {
            return (ptr_ != it.ptr_) || (wptr_ != it.wptr_);
        }
        int weight() const { return *wptr_; }
        int index() const { return ptr_ - base_; }
    };
    const_iterator begin() const {
        return const_iterator(vector_, weight_);
    }
    const_iterator end() const {
        return const_iterator(&vector_[size_], &weight_[size_]);
    }

    void print(std::ostream &os) const {
        os << '{';
        for( int i = 0; i < size_; ++i ) {
            os << vector_[i] << '@' << weight_[i] << ',';
        }
        os << '}';
    }
};

inline std::ostream& operator<<(std::ostream &os, const weighted_ordered_vector_t &vec) {
    vec.print(os);
    return os;
}

#undef DEBUG

#endif

