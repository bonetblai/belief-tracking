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


#ifndef GRID_VAR_BEAM_H
#define GRID_VAR_BEAM_H

#include <ordered_vector.h>
#include <cassert>
#include <iostream>
#include <vector>
#include <math.h>

inline int int_abs(int x) {
    return x >= 0 ? x : -x;
}

inline int int_pow(int base, int expn) {
    int result = 1;
    for( int i = 0; i < expn; ++i )
        result *= base;
    return result;
}

class grid_var_beam_t {
    int nvars_;
    int num_particles_;
    ordered_vector_t beam_;

    static int nrows_; 
    static int ncols_; 
    static int ncells_;

  public:
    grid_var_beam_t(int nvars)
      : nvars_(nvars), num_particles_(int_pow(ncells_, nvars_)) { }
    explicit grid_var_beam_t(const grid_var_beam_t &beam)
      : nvars_(beam.nvars_), num_particles_(beam.num_particles_),
        beam_(beam.beam_) { }
    grid_var_beam_t(grid_var_beam_t &&beam) = default;
    ~grid_var_beam_t() { }

    static void initialize(int nrows, int ncols) {
        static bool initialized = false;
        if( initialized && (nrows_ == nrows) && (ncols_ == ncols) ) return;
        initialized = true;

#if 0
        std::cout << "grid_var_beam_t: initialization:"
                  << " nrows=" << nrows
                  << ", ncols=" << ncols
                  << std::endl;
#endif

        nrows_ = nrows;
        ncols_ = ncols;
        ncells_ = nrows_ * ncols_;
    }

    static int nrows() { return nrows_; }
    static int ncols() { return ncols_; }

    enum { manhattan_neighbourhood = 0, octile_neighbourhood = 1 };
    static bool adjacent_cells(int row1, int col1, int cell, int neighbourhood, int radius) {
        int row2 = cell / ncols_, col2 = cell % ncols_;
        int abs_rdiff = int_abs(row1 - row2), abs_cdiff = int_abs(col1 - col2);
        if( neighbourhood == manhattan_neighbourhood ) {
            return ((abs_rdiff == 0) && (abs_cdiff <= radius)) ||
                   ((abs_cdiff == 0) && (abs_rdiff <= radius));
        } else {
            return (abs_rdiff <= radius) && (abs_cdiff <= radius);
        }
    }
    static bool adjacent_cells(int cell1, int cell2, int neighbourhood, int radius) {
        return adjacent_cells(cell1 / ncols_, cell1 % ncols_, cell2, neighbourhood, radius);
    }

    static int target_cell(int cell, int move) {
        switch( move ) {
          case 0:
            return cell + ncols_ >= ncells_ ? cell : cell + ncols_;
          case 1:
            return (cell % ncols_) == ncols_ - 1 ? cell : cell + 1;
          case 2:
            return cell - ncols_ < 0 ? cell : cell - ncols_;
          case 3:
            return (cell % ncols_) == 0 ? cell : cell - 1;
          case 4:
            return cell;
        }
        assert(0); // can't reach here
        return -1;
    }

    int value(int var, int p) const {
        for( ; var > 0; --var, p /= ncells_ );
        return p % ncells_;
    }

    bool consistent(int p) {
        for( int i = 0; i < nvars_; ++i ) {
            int val = value(i, p);
            for( int j = i+1; j < nvars_; ++j ) {
                if( val == value(j, p) ) return false;
            }
        }
        return true;
    }

    bool empty() const { return beam_.empty(); }
    int size() const { return beam_.size(); }
    bool known() const { return beam_.size() == 1; }
    bool contains(int value) const { return beam_.contains(value); }
    bool consistent() const { return !beam_.empty(); }

    void clear() { beam_.clear(); }
    void insert(int e) { beam_.insert(e); }
    void erase(int e) { beam_.erase(e); }
    void erase_ordered_indices(const std::vector<int> &indices) {
        beam_.erase_ordered_indices(indices);
    }

    void set_initial_configuration() {
        beam_.clear();
        for( int p = 0; p < num_particles_; ++p ) {
            if( consistent(p) ) {
                beam_.insert(p);
            }
        }
    }

    bool operator==(const grid_var_beam_t &beam) const {
        return beam_ == beam.beam_;
    }
    bool operator!=(const grid_var_beam_t &beam) const {
        return *this == beam ? false : true;
    }

    const grid_var_beam_t& operator=(const grid_var_beam_t &beam) {
        beam_ = beam.beam_;
        return *this;
    }

    void print(std::ostream &os) const {
        os << "{";
        for( const_iterator it = begin(); it != end(); ++it ) {
            int p = *it;
            os << "[p=" << p << ":";
            for( int var = 0; var < nvars_; ++var ) {
                int cell = value(var, p);
                os << "v" << var << "=(" << (cell % ncols_) << "," << (cell / ncols_) << "),";
            }
            os << "],";
        }
        os << "}";
    }

    typedef ordered_vector_t::const_iterator const_iterator;
    const_iterator begin() const { return beam_.begin(); }
    const_iterator end() const { return beam_.end(); }

    // determine if it is necessary that there is an object at or adjacent to
    // to given cell:
    //
    //   status_cell(cell, true, true) : checks if obj adjacent to cell
    //   status_cell(cell, true, false) : checks if obj at cell
    //   status_cell(cell, false, true) : checks if no obj adjacent to cell
    //   status_cell(cell, false, false) : checks if no obj at cell
    //
    // neighbourhood is used to determine adjacency: 0=manhattan, 1=octile
    // radius is used to determine the extension of surrounding region
    bool status_cell(int cell, bool check_obj, bool adjacent_to, int neighbourhood = manhattan_neighbourhood, int radius = 1) const {
        int row = cell / ncols_, col = cell % ncols_;
        for( const_iterator it = begin(); it != end(); ++it ) {
            int p = *it;
            bool found = false;
            for( int var = 0; var < nvars_; ++var ) {
                int var_value = value(var, p);
                if( (!adjacent_to && (var_value == cell)) ||
                    (adjacent_to && adjacent_cells(row, col, var_value, neighbourhood, radius)) ) {
                    found = true;
                    break;
                }
            }
            if( (check_obj && !found) || (!check_obj && found) ) return false;
        }
        return true;
    }
    bool necessary_obj_at(int cell) const { return status_cell(cell, true, false); }
    bool necessary_no_obj_at(int cell) const { return status_cell(cell, false, false); }
    bool possible_obj_at(int cell) const { return !necessary_no_obj_at(cell); }
    bool possible_no_obj_at(int cell) const { return !necessary_obj_at(cell); }
    bool necessary_obj_adjacent_to(int cell, int neighbourhood, int radius) const { return status_cell(cell, true, true, neighbourhood, radius); }
    bool necessary_no_obj_adjacent_to(int cell, int neighbourhood, int radius) const { return status_cell(cell, false, true, neighbourhood, radius); }
    bool possible_obj_adjacent_to(int cell, int neighbourhood, int radius) const { return !necessary_no_obj_adjacent_to(cell, neighbourhood, radius); }
    bool possible_no_obj_adjacent_to(int cell, int neighbourhood, int radius) const { return !necessary_obj_adjacent_to(cell, neighbourhood, radius); }

    // filter cell; parameters similar to status_cell
    void filter_cell(int cell, bool check_obj, bool adjacent_to, int neighbourhood = manhattan_neighbourhood, int radius = 1) {
        static std::vector<int> indices_to_erase;
        indices_to_erase.clear();
        indices_to_erase.reserve(beam_.size());
        int row = cell / ncols_, col = cell % ncols_;
        for( const_iterator it = begin(); it != end(); ++it ) {
            int p = *it;
            bool found = false;
            for( int var = 0; var < nvars_; ++var ) {
                int var_value = value(var, p);
                if( (!adjacent_to && (var_value == cell)) ||
                    (adjacent_to && adjacent_cells(row, col, var_value, neighbourhood, radius)) ) {
                    found = true;
                    break;
                }
            }
            if( (check_obj && !found) || (!check_obj && found) )
                indices_to_erase.push_back(it.index());
        }
        beam_.erase_ordered_indices(indices_to_erase);
    }
    void filter_obj_at(int cell) { filter_cell(cell, true, false); }
    void filter_no_obj_at(int cell) { filter_cell(cell, false, false); }
    void filter_obj_adjacent_to(int cell, int neighbourhood, int radius) { filter_cell(cell, true, true, neighbourhood, radius); }
    void filter_no_obj_adjacent_to(int cell, int neighbourhood, int radius) { filter_cell(cell, false, true, neighbourhood, radius); }

    void recursive_move_non_det(int p, int q, int m, int var, ordered_vector_t &beam) {
        if( var == nvars_ ) {
            if( consistent(q) ) beam.insert(q);
        } else {
            int cell = p % ncells_;
            int np = p / ncells_;
            int nm = m * ncells_;
            for( int i = 0; i < 5; ++i ) {
                int ncell = target_cell(cell, i);
                int nq = q + (ncell * m);
                recursive_move_non_det(np, nq, nm, 1+var, beam);
            }
        }
    }

    void move_non_det() {
        static ordered_vector_t tmp;
        tmp.reserve(int_pow(5, nvars_));
        ordered_vector_t nbeam; // can't be static because of the std::move()
        for( const_iterator it = begin(); it != end(); ++it ) {
            int p = *it;
            recursive_move_non_det(p, 0, 1, 0, tmp);
            nbeam.insert(tmp);
            tmp.clear();
        }
        //beam_ = std::move(nbeam);
        beam_ = nbeam; // CHECK: replace this by std::move()
    }
};

int grid_var_beam_t::nrows_ = 0;
int grid_var_beam_t::ncols_ = 0;
int grid_var_beam_t::ncells_ = 0;

inline std::ostream& operator<<(std::ostream &os, const grid_var_beam_t &beam) {
    beam.print(os);
    return os;
}

#endif

