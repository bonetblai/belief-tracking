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

#ifndef BASE_BELIEF_H
#define BASE_BELIEF_H

#include <var_beam.h>
#include <arc_consistency.h>

#include <cassert>
#include <iostream>
#include <vector>


namespace Wumpus {

namespace Diagonal {

struct position_t {
    int col_, row_;
    position_t() : col_(0), row_(0) { }
    position_t(int dim, int index) {
        assert((index >= 0) && (index < 3 * dim - 5));
        if( index < dim - 1 ) { // diagonal beams
            col_ = 1 + index;
            row_ = 1 + index;
        } else if( index < 2 * dim - 3) { // above-diagonal beams
            col_ = 0 + index - dim + 1;
            row_ = 2 + index - dim + 1;
        } else { // below-diagonal beams
            col_ = 2 + index - 2 * dim + 3;
            row_ = 0 + index - 2 * dim + 3;
        }
    }
    void print(std::ostream &os) const {
        os << "col=" << col_ << ",row=" << row_;
    }
};

struct valuation_t {
    bool n_wumpus_;
    bool e_wumpus_;
    bool s_wumpus_;
    bool w_wumpus_;

    valuation_t(int p = 0) {
        assert((p >= 0) && (p < 16));
        n_wumpus_ = (p % 2) == 1;
        p = p >> 1;
        e_wumpus_ = (p % 2) == 1;
        p = p >> 1;
        s_wumpus_ = (p % 2) == 1;
        p = p >> 1;
        w_wumpus_ = (p % 2) == 1;
    }
    valuation_t(bool n_wumpus, bool e_wumpus, bool s_wumpus, bool w_wumpus)
      : n_wumpus_(n_wumpus), e_wumpus_(e_wumpus),
        s_wumpus_(s_wumpus), w_wumpus_(w_wumpus) {
    }

    int encode() const {
        int p = w_wumpus_ ? 1 : 0;
        p = 2 * p + (s_wumpus_ ? 1 : 0);
        p = 2 * p + (e_wumpus_ ? 1 : 0);
        p = 2 * p + (n_wumpus_ ? 1 : 0);
        return p;
    }

    void print(std::ostream &os) const {
        os << "[n_wumpus=" << (n_wumpus_ ? 1 : 0)
           << ",e_wumpus=" << (e_wumpus_ ? 1 : 0)
           << ",s_wumpus=" << (s_wumpus_ ? 1 : 0)
           << ",w_wumpus=" << (w_wumpus_ ? 1 : 0)
           << "]";
    }
};

};

};

inline std::ostream& operator<<(std::ostream &os, const Wumpus::Diagonal::position_t &p) {
    p.print(os);
    return os;
}

inline std::ostream& operator<<(std::ostream &os, const Wumpus::Diagonal::valuation_t &p) {
    p.print(os);
    return os;
}


namespace Wumpus {

namespace Diagonal {

class arc_consistency_t : public CSP::arc_consistency_t<var_beam_t> {
    static int dim_;
    mutable position_t x_pos_, y_pos_;
    mutable valuation_t x_val_;

    // disallow copy constructor
    explicit arc_consistency_t(const arc_consistency_t &);
    explicit arc_consistency_t(arc_consistency_t &&);

  public:
    arc_consistency_t();
    virtual ~arc_consistency_t() { }

    static void initialize(int dim) {
        dim_ = dim;
    }

    virtual void arc_reduce_preprocessing_0(int var_x, int var_y) {
        assert((var_x >= 0) && (var_x < 3 * dim_ - 5));
        x_pos_ = position_t(dim_, var_x);
        y_pos_ = position_t(dim_, var_y);
#ifdef DEBUG
        std::cout << "Preparing: X=cell(" << x_pos_ << ")" << ", Y=cell(" << y_pos_ << ")"
                  << std::endl;
#endif
    }
    virtual void arc_reduce_preprocessing_1(int var_x, int val_x) {
        x_val_ = valuation_t(val_x);
    }
    virtual void arc_reduce_postprocessing(int var_x, int var_y) {
    }

    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const {
        valuation_t y_val_(val_y);
        if( x_pos_.col_ < y_pos_.col_ ) {
            if( x_pos_.row_ < y_pos_.row_ ) {
                assert((x_pos_.col_ + 1 == y_pos_.col_) && (x_pos_.row_ + 1 == y_pos_.row_));
                return (x_val_.n_wumpus_ == y_val_.w_wumpus_) &&
                       (x_val_.e_wumpus_ == y_val_.s_wumpus_);
            } else if( x_pos_.row_ == y_pos_.row_ ) {
                assert(2 + x_pos_.col_ == y_pos_.col_ );
                return x_val_.e_wumpus_ == y_val_.w_wumpus_;
            } else {
                assert((x_pos_.col_ + 1 == y_pos_.col_) && (x_pos_.row_ == 1 + y_pos_.row_));
                return (x_val_.e_wumpus_ == y_val_.n_wumpus_) &&
                       (x_val_.s_wumpus_ == y_val_.w_wumpus_);
            }
        } else if( x_pos_.col_ == y_pos_.col_ ) {
            if( x_pos_.row_ < y_pos_.row_ ) {
                assert(2 + x_pos_.row_ == y_pos_.row_);
                return x_val_.n_wumpus_ == y_val_.s_wumpus_;
            } else {
                assert(x_pos_.row_ == 2 + y_pos_.row_);
                return x_val_.s_wumpus_ == y_val_.n_wumpus_;
            }
        } else {
            if( x_pos_.row_ < y_pos_.row_ ) {
                assert((x_pos_.col_ == 1 + y_pos_.col_) && (x_pos_.row_ + 1 == y_pos_.row_));
                return (x_val_.n_wumpus_ == y_val_.e_wumpus_) &&
                       (x_val_.w_wumpus_ == y_val_.s_wumpus_);
            } else if( x_pos_.row_ == y_pos_.row_ ) {
                assert(x_pos_.col_ == 2 + y_pos_.col_);
                return x_val_.w_wumpus_ == y_val_.e_wumpus_;
            } else {
                assert((x_pos_.col_ == 1 + y_pos_.col_) && (x_pos_.row_ == 1 + y_pos_.row_));
                return (x_val_.s_wumpus_ == y_val_.e_wumpus_) &&
                       (x_val_.w_wumpus_ == y_val_.n_wumpus_);
            }
        }
    }

    const arc_consistency_t& operator=(const arc_consistency_t &arc) {
        assert(0); // shouldn't be called
        return arc;
    }
    bool operator==(const arc_consistency_t &arc) {
        assert(0); // shouldn't be called
        return false;
    }
};

int arc_consistency_t::dim_ = 0;

class base_belief_t {
  protected:
    static int dim_;
    static CSP::constraint_digraph_t cg_;

  public:
    base_belief_t() { }
    base_belief_t(const base_belief_t &bel) { }
#if 0
    base_belief_t(base_belief_t &&bel) { }
#endif
    virtual ~base_belief_t() { }

    static void initialize(int dim) {
        static bool initialized = false;
        if( initialized && (dim_ == dim) ) return;
        initialized = true;
        dim_ = dim;

#if 0
        std::cout << "base_belief_t: initialization: "
                  << "dim=" << dim
                  << std::endl;
#endif

        // constraint graph
        construct_constraint_graph(dim_);
        arc_consistency_t::initialize(dim_);
    }

    static int dim() { return dim_; }
    static const CSP::constraint_digraph_t& cg() { return cg_; }

    virtual const base_belief_t& operator=(const base_belief_t &bel) {
        return *this;
    }

    virtual bool operator==(const base_belief_t &bel) const {
        return dim_ == bel.dim_;
    }
    virtual bool operator!=(const base_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    virtual void print(std::ostream &os) const = 0;

    // Consistency methods
    static void construct_constraint_graph(int dim) {
        cg_.create_empty_graph(3 * dim - 5);
        for( int p = 0; p < 3 * dim - 5; ++p ) {
            cg_.reserve_edge_list(p, 8);
        }
        for( int x = 0; x < 3 * dim - 5; ++x ) {
            position_t x_pos(dim_, x);
            for( int y = 0; y < 3 * dim - 5; ++y ) {
                position_t y_pos(dim_, y);
                int abs_col = x_pos.col_ - y_pos.col_;
                abs_col = abs_col < 0 ? -abs_col : abs_col;
                int abs_row = x_pos.row_ - y_pos.row_;
                abs_row = abs_row < 0 ? -abs_row : abs_row;
                if( ((abs_col == 1) && (abs_row == 1)) ||
                    ((abs_col == 0) && (abs_row == 2)) ||
                    ((abs_row == 0) && (abs_col == 2)) ) {
                        cg_.add_edge(x, y);
                }
            }
        }
        //std::cout << "cg: #edges=" << cg_.nedges() << std::endl;
    }
};

int base_belief_t::dim_ = 0;
CSP::constraint_digraph_t base_belief_t::cg_;

arc_consistency_t::arc_consistency_t()
  : CSP::arc_consistency_t<var_beam_t>(base_belief_t::cg()) {
}

};

};

#endif

