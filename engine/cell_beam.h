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


/*
 * This type of beam is used in grid problems where each cell in the
 * grid is a boolean variable that denote the presence of an object,
 * and the variables are pairwise independent a priori.  Each beam 
 * consists of the valuations for the variables for the cell and the
 * neighbouring cells, where the neighbourhood is determined by a
 * subset of adjancet cells.
 *
 */


#ifndef CELL_BEAM_H
#define CELL_BEAM_H

#include "ordered_vector.h"

#include <cassert>
#include <iostream>
#include <vector>
#include <math.h>

class cell_beam_t {
    int row_;
    int col_;
    int type_;
    ordered_vector_t beam_;

    static int virgin_size_[];
    static int num_objs_[];

  public:
    cell_beam_t(int row = 0, int col = 0, int type = 0)
      : row_(row), col_(col), type_(type) { }
    explicit cell_beam_t(const cell_beam_t &beam)
      : row_(beam.row_), col_(beam.col_), type_(beam.type_),
        beam_(beam.beam_) { }
    cell_beam_t(cell_beam_t &&beam) = default;
    ~cell_beam_t() { }

    enum { TOP = 1, BOTTOM = 2, LEFT = 4, RIGHT = 8 };

    static void initialize() {
        static bool initialized = false;
        if( initialized ) return;
        initialized = true;

        //std::cout << "cell_beam_t: initialization" << std::endl;
        for( int p = 0; p < 512; ++p ) {
            int num = 0;
            for( int q = p; q != 0; q = q >> 1 ) {
                num += (q & 0x1);
            }
            num_objs_[p] = num;
        }

        for( int type = 0; type < 16; ++type ) {
            int num = -1;
            switch( type ) {
                case 0: num = 512; break;
                case 1:
                case 2:
                case 4:
                case 8: num = 64; break;
                case 5:
                case 6:
                case 9:
                case 10: num = 16; break;
            }
            virgin_size_[type] = num;
        }
    }

    int row() const { return row_; }
    int col() const { return col_; }
    bool empty() const { return beam_.empty(); }
    int size() const { return beam_.size(); }

    void clear() { beam_.clear(); }

    // determine (for sure) the status of the object at beam's cell
    bool status_obj_at(int status) const {
        for( ordered_vector_t::const_iterator it = beam_.begin(); it != beam_.end(); ++it ) {
            int p = *it;
            if( ((p >> 4) & 0x1) != status ) return false;
        }
        return true;
    }
    bool obj_at() const { return !empty() && status_obj_at(1); }
    bool no_obj_at() const { return !empty() && status_obj_at(0); }

    // determine max number of objects sorrounding this cell
    std::pair<int, int> num_surrounding_objs() const {
        int min_nobjs = 9, max_nobjs = 0;
        for( ordered_vector_t::const_iterator it = beam_.begin(); it != beam_.end(); ++it ) {
            int p = *it;
            if( p & 0x10 ) continue;
            int nobjs = num_objs_[p];
            min_nobjs = nobjs < min_nobjs ? nobjs : min_nobjs;
            max_nobjs = nobjs > max_nobjs ? nobjs : max_nobjs;
        }
        return std::make_pair(min_nobjs, max_nobjs);
    }

    float obj_probability(float prior, int bit_index) const {
        float mass = 0, total = 0;
        for( ordered_vector_t::const_iterator it = beam_.begin(); it != beam_.end(); ++it ) {
            int p = *it;
            //int n = num_objs_[p];
            //float prob = powf(prior, n) * powf(1.0 - prior, 9 - n);
            if( (p >> bit_index) & 0x1 ) {
                //mass += prob;
                mass += 1;
            }
            //total += prob;
            total += 1;
        }
        return mass / total;
    }

    bool virgin() const { return beam_.size() == virgin_size_[type_]; }

    void erase_ordered_indices(const std::vector<int> &indices) {
        beam_.erase_ordered_indices(indices);
    }

    void set_initial_configuration(int neighbourhood) {
        beam_.clear();
        for( int p = 0; p < 512; ++p ) {
            int q = p & neighbourhood;
            if( (type_ & TOP) && ((q & 0x40) || (q & 0x80) || (q & 0x100)) ) continue;
            if( (type_ & BOTTOM) && ((q & 0x1) || (q & 0x2) || (q & 0x4)) ) continue;
            if( (type_ & LEFT) && ((q & 0x1) || (q & 0x8) || (q & 0x40)) ) continue;
            if( (type_ & RIGHT) && ((q & 0x4) || (q & 0x20) || (q & 0x100)) ) continue;
            beam_.insert(q);
        }
    }

    void insert(int e) { beam_.insert(e); }

    void filter(int nobjs, bool at_least = false) {
        assert((0 <= nobjs) && (nobjs <= 9));
        static std::vector<int> indices_to_erase;
        indices_to_erase.clear();
        indices_to_erase.reserve(beam_.size());
        for( ordered_vector_t::const_iterator it = beam_.begin(); it != beam_.end(); ++it ) {
            int p = *it;
            if( nobjs == 9 ) {
                if( (p & 0x10) == 0 )
                    indices_to_erase.push_back(it.index());
            } else {
                if( (p & 0x10) ||
                    (!at_least && (num_objs_[p] != nobjs)) ||
                    (at_least && (num_objs_[p] < nobjs)) )
                    indices_to_erase.push_back(it.index());
            }
        }
        beam_.erase_ordered_indices(indices_to_erase);
    }

    bool operator==(const cell_beam_t &beam) const {
        return (row_ == beam.row_) && (col_ == beam.col_) && (beam_ == beam.beam_);
    }
    bool operator!=(const cell_beam_t &beam) const {
        return *this == beam ? false : true;
    }

    const cell_beam_t& operator=(const cell_beam_t &beam) {
        row_ = beam.row_;
        col_ = beam.col_;
        type_ = beam.type_;
        beam_ = beam.beam_;
        return *this;
    }

    void print(std::ostream &os) const {
        os << beam_;
    }

    typedef ordered_vector_t::const_iterator const_iterator;
    const_iterator begin() const { return beam_.begin(); }
    const_iterator end() const { return beam_.end(); }
};

int cell_beam_t::virgin_size_[16];
int cell_beam_t::num_objs_[512];

inline std::ostream& operator<<(std::ostream &os, const cell_beam_t &beam) {
    beam.print(os);
    return os;
}

#endif

