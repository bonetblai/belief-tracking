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

#ifndef BATTLESHIP_BASE_BELIEF_H
#define BATTLESHIP_BASE_BELIEF_H

#include "var_beam.h"
#include "arc_consistency.h"
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <vector>

//#define DEBUG

namespace Battleship {

struct placement_t {
    static int max_ship_size_;

    bool hit_;
    bool horiz_;
    int anchor_;
    int size_;
    int nhits_;

    enum { empty_cell = 0, empty_cell_hit = 1 };

    placement_t(int p = 0) {
        hit_ = p % 2 == 0 ? false : true;
        p = p >> 1;
        horiz_ = p % 2 == 0 ? false : true;
        p = p >> 1;
        anchor_ = p % max_ship_size_;
        p = p / max_ship_size_;
        size_ = p % (1 + max_ship_size_);
        nhits_ = p / (1 + max_ship_size_);
    }
    placement_t(bool hit, bool horiz, int anchor, int size, int nhits)
      : hit_(hit), horiz_(horiz), anchor_(anchor), size_(size), nhits_(nhits) {
    }

    static void initialize(int max_ship_size) {
        max_ship_size_ = max_ship_size;
    }

    int encode() const {
        int p = nhits_;
        p = p * (1 + max_ship_size_) + size_;
        p = p * max_ship_size_ + anchor_;
        p = p * 2 + (horiz_ ? 1 : 0);
        p = p * 2 + (hit_ ? 1 : 0);
        return p;
    }

    void print(std::ostream &os) const {
        os << "[hit=" << (hit_ ? "T" : "F")
           << ",hz=" << (horiz_ ? "T" : "F")
           << ",anc=" << anchor_
           << ",sz=" << size_
           << ",n=" << nhits_
           << "]";
    }
};

}; // end of namespace Battleship

inline std::ostream& operator<<(std::ostream &os, const Battleship::placement_t &p) {
    p.print(os);
    return os;
}

namespace Battleship {

class arc_consistency_t : public CSP::arc_consistency_t<var_beam_t> {
    int ncols_;
    bool allow_adjacent_ships_;

    mutable int x_row_, x_col_, x_plus_;
    mutable int y_row_, y_col_;
    mutable placement_t x_plc_;

    // disallow copy/move constructor
    explicit arc_consistency_t(const arc_consistency_t &);
    explicit arc_consistency_t(arc_consistency_t &&);

  public:
    arc_consistency_t(int ncols, bool allow_adjacent_ships);
    virtual ~arc_consistency_t() { }

    virtual void arc_reduce_preprocessing_0(int var_x, int var_y) {
        x_col_ = var_x % ncols_;
        x_row_ = var_x / ncols_;
        y_col_ = var_y % ncols_;
        y_row_ = var_y / ncols_;
        assert(!allow_adjacent_ships_ || (x_col_ == y_col_) || (x_row_ == y_row_));
        assert((x_col_ != y_col_) || (x_row_ != y_row_));

        // assertions
        if((x_col_ != y_col_) && (x_row_ != y_row_)) {
            assert(std::abs(x_col_ - y_col_) < 2);
            assert(std::abs(x_row_ - y_row_) < 2);
        }

#ifdef DEBUG
        std::cout << "Preparing: X=cell(" << x_col_ << "," << x_row_ << ")"
                  << ", Y=cell(" << y_col_ << "," << y_row_ << ")"
                  << std::endl;
#endif
    }
    virtual void arc_reduce_preprocessing_1(int var_x, int val_x) {
        x_plc_ = placement_t(val_x);
        x_plus_ = x_plc_.size_ == 0 ? 0 : x_plc_.size_ - x_plc_.anchor_ - 1;
    }
    virtual void arc_reduce_postprocessing(int var_x, int var_y) {
    }

    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const;

    const arc_consistency_t& operator=(const arc_consistency_t &arc) {
        assert(0); // shouldn't be called
        return arc;
    }
    bool operator==(const arc_consistency_t &arc) {
        assert(0); // shouldn't be called
        return false;
    }
};

class base_belief_t {
  public:
    static int nrows_;
    static int ncols_;
    static int ncells_;
    static int max_ship_size_;
    static bool allow_adjacent_ships_;
    static int effective_max_ship_size_;
    static int num_particles_;
    static CSP::constraint_digraph_t cg_;

  public:
    base_belief_t() { }
    base_belief_t(const base_belief_t &bel) { }
    base_belief_t(base_belief_t &&bel) { }
    virtual ~base_belief_t() { }

    static void initialize(int nrows, int ncols, int max_ship_size, const int *ship_inventory, bool allow_adjacent_ships) {
        nrows_ = nrows;
        ncols_ = ncols;
        ncells_ = ncols_ * nrows_;
        max_ship_size_ = max_ship_size;
        allow_adjacent_ships_ = allow_adjacent_ships;

        effective_max_ship_size_ = max_ship_size;
        while( ship_inventory[effective_max_ship_size_] == 0 )
            --effective_max_ship_size_;

        num_particles_ = 4 * effective_max_ship_size_ * (1 + effective_max_ship_size_) * (1 + effective_max_ship_size_);
        //construct_constraint_graph(nrows_, ncols_, effective_max_ship_size_);
        construct_constraint_graph_simplified(nrows_, ncols_, effective_max_ship_size_);
        placement_t::initialize(effective_max_ship_size_);

        std::cout << "base_belief_t:"
                  << " effective-max-ship-size = " << effective_max_ship_size_
                  << ", num-particles = " << num_particles_
                  << std::endl;
    }

    int nrows() const { return nrows_; }
    int ncols() const { return ncols_; }
    int max_ship_size() const { return max_ship_size_; }
    int effective_max_ship_size() const { return effective_max_ship_size_; }
    static const CSP::constraint_digraph_t& cg() { return cg_; }

    virtual const base_belief_t& operator=(const base_belief_t &bel) = 0;
    virtual bool operator==(const base_belief_t &bel) const = 0;
    virtual bool operator!=(const base_belief_t &bel) const = 0;
    virtual void print(std::ostream &os) const = 0;

    // Consistency methods
    static void construct_constraint_graph(int nrows, int ncols, int max_ship_size) {
        cg_.create_empty_graph(nrows * ncols);
        for( int cell_0 = 0; cell_0 < ncells_; ++cell_0 ) {
            cg_.reserve_edge_list(cell_0, ncells_ - 1);
            for( int cell_1 = 0; cell_1 < ncells_; ++cell_1 ) {
                if( cell_0 != cell_1 ) {
                    int row_0 = cell_0 / ncols_;
                    int col_0 = cell_0 % ncols_;
                    int row_1 = cell_1 / ncols_;
                    int col_1 = cell_1 % ncols_;
                    if( ((row_0 == row_1) && (std::abs(col_0 - col_1) < max_ship_size)) ||
                        ((col_0 == col_1) && (std::abs(row_0 - row_1) < max_ship_size)) ) {
                        cg_.add_edge(cell_1, cell_0);
                    }
                }
            }
        }
        std::cout << "cg: #vars=" << cg_.nvars() << ", #edges=" << cg_.nedges() << std::endl;
    }

    static void construct_constraint_graph_simplified(int nrows, int ncols, int max_ship_size) {
        cg_.create_empty_graph(nrows * ncols);
        for( int cell = 0; cell < ncells_; ++cell ) {
            cg_.reserve_edge_list(cell, 8);
            int r = cell / ncols_, c = cell % ncols_;
            for( int dr = -1; dr <= 1; ++dr ) {
                if( (r + dr < 0) || (r + dr >= nrows_) ) continue;
                for( int dc = -1; dc <= 1; ++dc ) {
                    if( (c + dc < 0) || (c + dc >= ncols_) ) continue;
                    if( (dr == 0) && (dc == 0) ) continue;
                    if( allow_adjacent_ships_ && (dr != 0) && (dc != 0) ) continue;
                    int other_cell = (r + dr) * ncols_ + (c + dc);
                    cg_.add_edge(cell, other_cell);
                }
            }
        }
        std::cout << "cg: #vars=" << cg_.nvars() << ", #edges=" << cg_.nedges() << std::endl;
    }
};

int placement_t::max_ship_size_ = 0;

int base_belief_t::nrows_ = 0;
int base_belief_t::ncols_ = 0;
int base_belief_t::ncells_ = 0;
int base_belief_t::max_ship_size_ = 0;
bool base_belief_t::allow_adjacent_ships_ = true;
int base_belief_t::effective_max_ship_size_ = 0;
int base_belief_t::num_particles_ = 0;

CSP::constraint_digraph_t base_belief_t::cg_;

arc_consistency_t::arc_consistency_t(int ncols, bool allow_adjacent_ships)
  : CSP::arc_consistency_t<var_beam_t>(base_belief_t::cg()),
    ncols_(ncols), allow_adjacent_ships_(allow_adjacent_ships) {
}

bool arc_consistency_t::consistent(int var_x, int var_y, int val_x, int val_y) const {
    placement_t y_plc(val_y);

#ifdef DEBUG
    std::cout << "    val_X=" << val_x << "=" << x_plc_
              << ", val_Y=" << val_y << "=" << y_plc
              << ": consistent="
              << std::flush;
#endif

    // diagonal placements (only when allow_adjacent_ships is false)
    if( !allow_adjacent_ships_ ) {
        if( (x_col_ != y_col_) && (x_row_ != y_row_) ) {
            return (x_plc_.size_ == 0) || (y_plc.size_ == 0);
        }
    }

    int gap = allow_adjacent_ships_ || (x_plc_.size_ == 0) || (y_plc.size_ == 0) ? 0 : 1;
    int y_plus = y_plc.size_ == 0 ? 0 : y_plc.size_ - y_plc.anchor_ - 1;

    // non-overlapping (disjoint) placements
    if( x_plc_.horiz_ ) { // horizontal placement for x_plc
        if( x_col_ < y_col_ ) {
            assert(x_row_ == y_row_);
            if( (x_col_ + x_plus_ + gap < y_col_) &&
                (!y_plc.horiz_ || (y_col_ - y_plc.anchor_ - gap > x_col_)) ) {
#ifdef DEBUG
                std::cout << "T (disjoint placement)" << std::endl;
#endif
                return true;
            }
        } else if( x_col_ == y_col_ ) {
            assert(x_row_ != y_row_);
            if( y_plc.horiz_ ) {
#ifdef DEBUG
                std::cout << "T (disjoint placement)" << std::endl;
#endif
                return true;
            } else if( x_row_ < y_row_ ) {
                if( y_row_ - y_plc.anchor_ - gap > x_row_ ) {
#ifdef DEBUG
                    std::cout << "T (disjoint placement)" << std::endl;
#endif
                    return true;
                }
            } else {
                assert(x_row_ > y_row_);
                if( y_row_ + y_plus + gap < x_row_ ) {
#ifdef DEBUG
                    std::cout << "T (disjoint placement)" << std::endl;
#endif
                    return true;
                }
            }
        } else {
            assert(x_col_ > y_col_);
            assert(x_row_ == y_row_);
            if( (x_col_ - x_plc_.anchor_ - gap > y_col_) &&
                (!y_plc.horiz_ || (y_col_ + y_plus + gap < x_col_)) ) {
#ifdef DEBUG
                std::cout << "T (disjoint placement)" << std::endl;
#endif
                return true;
            }
        }
    } else { // vertical placement for x_plc
        if( x_row_ < y_row_ ) {
            assert(x_col_ == y_col_);
            if( (x_row_ + x_plus_ + gap < y_row_) &&
                (y_plc.horiz_ || (y_row_ - y_plc.anchor_ - gap > x_row_)) ) {
#ifdef DEBUG
                std::cout << "T (disjoint placement)" << std::endl;
#endif
                return true;
            }
        } else if( x_row_ == y_row_ ) {
            assert(x_col_ != y_col_);
            if( !y_plc.horiz_ ) {
#ifdef DEBUG
                std::cout << "T (disjoint placement)" << std::endl;
#endif
                return true;
            } else if( x_col_ < y_col_ ) {
                if( y_col_ - y_plc.anchor_ - gap > x_col_ ) {
#ifdef DEBUG
                    std::cout << "T (disjoint placement)" << std::endl;
#endif
                    return true;
                }
            } else {
                assert(x_col_ > y_col_);
                if( y_col_ + y_plus + gap < x_col_ ) {
#ifdef DEBUG
                    std::cout << "T (disjoint placement)" << std::endl;
#endif
                    return true;
                }
            }
        } else {
            assert(x_row_ > y_row_);
            assert(x_col_ == y_col_);
            if( (x_row_ - x_plc_.anchor_ - gap > y_row_) &&
                (y_plc.horiz_ || (y_row_ + y_plus + gap < x_row_)) ) {
#ifdef DEBUG
                std::cout << "T (disjoint placement)" << std::endl;
#endif
                return true;
            }
        }
    }

    // overlapping placements
    if( (x_plc_.size_ == y_plc.size_) &&
        (x_plc_.horiz_ == y_plc.horiz_) && (x_plc_.nhits_ == y_plc.nhits_) ) {
        if( x_plc_.horiz_ && (x_row_ == y_row_) ) {
            if( x_col_ - x_plc_.anchor_ == y_col_ - y_plc.anchor_ ) {
#ifdef DEBUG
                std::cout << "T (compatible horizontal ships)" << std::endl;
#endif
                return true;
            }
        } else if( !x_plc_.horiz_ && (x_col_ == y_col_) ) {
            if( x_row_ - x_plc_.anchor_ == y_row_ - y_plc.anchor_ ) {
#ifdef DEBUG
                std::cout << "T (compatible vertical ships)" << std::endl;
#endif
                return true;
            }
        }
    }

#ifdef DEBUG
    std::cout << "F" << std::endl;
#endif

    return false;
}

}; // end of namespace Battleship

#undef DEBUG

#endif

