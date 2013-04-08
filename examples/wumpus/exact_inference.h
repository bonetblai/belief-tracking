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

#ifndef EXACT_INFERENCE_H
#define EXACT_INFERENCE_H

/*
#include "problem.h"
#include "base_policy.h"
#include <vector>
*/

#include "defs.h"

#include <cassert>
#include <iostream>
#include <fstream>

//#define CMD_FOR_SAT_SOLVER      "(RESULT=`minisat theory.cnf | tail -1`; echo $RESULT; if [ \"$RESULT\" = \"SATISFIABLE\" ]; then exit 0; else exit 1; fi)"
#define CMD_FOR_SAT_SOLVER      "(RESULT=`minisat theory.cnf | tail -1`; if [ \"$RESULT\" = \"SATISFIABLE\" ]; then exit 0; else exit 1; fi)"

/*
 * Exact inference by calling a SAT solver. Propositional variables
 * are 4nm where n is nrows and m is ncols. So, 4 variables per cell
 * that denote whether the cell has wumpus, pit, stench or breeze.
 *
 * Minisat should be in the path for this module to work correctly.
 *
 */

struct exact_inference_t {
    int nrows_;
    int ncols_;
    int ncells_;
    int offset_[4];

    mutable int ncls_extra_;

    enum { Breeze = 0, Stench = 1, Wumpus = 2, Pit = 3 };

    void process_sensed_information(const char *sensed_info,
                                    std::vector<int> &unvisited_cells_at_fringe) const {
        ncls_extra_ = 0;
        unvisited_cells_at_fringe.clear();
        unvisited_cells_at_fringe.reserve(ncells_);
        for( int p = 0; p < ncells_; ++p ) {
            if( sensed_info[p] == -1 ) {
                // This is an unvisited cell. If at fringe, insert it.
                if( (p + ncols_ < ncells_) && (sensed_info[p + ncols_] != -1) ) {
                    unvisited_cells_at_fringe.push_back(p);
                } else if( (p + 1 < ncells_) && (p % ncols_ < ncols_ - 1) && (sensed_info[p + 1] != -1) ) {
                    unvisited_cells_at_fringe.push_back(p);
                } else if( (p >= ncols_) && (sensed_info[p - ncols_] != -1) ) {
                    unvisited_cells_at_fringe.push_back(p);
                } else if( (p > 0) && (p % ncols_ > 0) && (sensed_info[p - 1] != -1) ) {
                    unvisited_cells_at_fringe.push_back(p);
                }
            } else {
                ncls_extra_ += 4;
            }
        }
    }

    void cls_for_surrounding_cells(std::ostream &os, int lit, int p, bool positive, int type, bool single_cls) const {
        int r = p / ncols_, c = p % ncols_;
        bool have_body = false;
        if( r < nrows_ - 1 ) {
            if( !single_cls || (single_cls && !have_body) ) { os << lit; have_body = true; }
            os << (positive ? " " : " -") << offset_[type] + ((r + 1) * ncols_ + c);
            if( !single_cls ) { os << " 0" << std::endl; }
        }
        if( c < ncols_ - 1 ) {
            if( !single_cls || (single_cls && !have_body) ) { os << lit; have_body = true; }
            os << (positive ? " " : " -") << offset_[type] + (r * ncols_ + c + 1);
            if( !single_cls ) { os << " 0" << std::endl; }
        }
        if( r > 0 ) {
            if( !single_cls || (single_cls && !have_body) ) { os << lit; have_body = true; }
            os << (positive ? " " : " -") << offset_[type] + ((r - 1) * ncols_ + c);
            if( !single_cls ) { os << " 0" << std::endl; }
        }
        if( c > 0) {
            if( !single_cls || (single_cls && !have_body) ) { os << lit; have_body = true; }
            os << (positive ? " " : " -") << offset_[type] + (r * ncols_ + c - 1);
            if( !single_cls ) { os << " 0" << std::endl; }
        }
        if( single_cls ) { os << " 0" << std::endl; }
    }

    void header(std::ostream &os, int ncls_extra = 0) const {
        int nvars = 4 * ncells_;
        int ncls = 10 * ncells_ - 4 * (nrows_ + ncols_) + ncls_extra;
        os << "p cnf " << nvars << " " << ncls << std::endl;
    }

    void cls_for_base_theory(std::ostream &os) const {
        // For each cell:
        //   * if it contains a wumpus, the surrounding cells sense stench
        //   * if it contains a pit, the surrounding cells sense breeze
        //   * if it doesn't sense stench, pit, the surrounding cells contain no wumpus
        //   * if it doesn't sense breeze, pit, the surrounding cells contain no pits
        for( int p = 0; p < ncells_; ++p ) {
            cls_for_surrounding_cells(os, -(offset_[Wumpus] + p), p, true, Stench, true);
            cls_for_surrounding_cells(os, -(offset_[Pit] + p), p, true, Breeze, true);
            cls_for_surrounding_cells(os, offset_[Stench] + p, p, false, Wumpus, false);
            cls_for_surrounding_cells(os, offset_[Breeze] + p, p, false, Pit, false);
        }
    }

    void other_cls(std::ostream &os, const char *sensed_info, int query) const {
        for( int p = 0; p < ncells_; ++p ) {
            if( sensed_info[p] != -1 ) {
                //std::cout << "Visit@(" << p/ncols_ << "," << p%ncols_ << ")" << std::endl;
                os << -(offset_[Wumpus] + p) << " 0" << std::endl;
                os << -(offset_[Pit] + p) << " 0" << std::endl;
                if( !(sensed_info[p] & Wumpus::Breeze) && !(sensed_info[p] & Wumpus::Stench) ) {
                    os << -(offset_[Breeze] + p) << " 0" << std::endl;
                    os << -(offset_[Stench] + p) << " 0" << std::endl;
                } else if( (sensed_info[p] & Wumpus::Breeze) && (sensed_info[p] & Wumpus::Stench) ) {
                    //std::cout << "Breeze@(" << p/ncols_ << "," << p%ncols_ << ")" << std::endl;
                    //std::cout << "Stench@(" << p/ncols_ << "," << p%ncols_ << ")" << std::endl;
                    os << (offset_[Breeze] + p) << " 0" << std::endl;
                    os << (offset_[Stench] + p) << " 0" << std::endl;
                } else if( sensed_info[p] & Wumpus::Breeze ) {
                    //std::cout << "Breeze@(" << p/ncols_ << "," << p%ncols_ << ")" << std::endl;
                    os << (offset_[Breeze] + p) << " 0" << std::endl;
                    os << -(offset_[Stench] + p) << " 0" << std::endl;
                } else if( sensed_info[p] & Wumpus::Stench ) {
                    //std::cout << "Stench@(" << p/ncols_ << "," << p%ncols_ << ")" << std::endl;
                    os << -(offset_[Breeze] + p) << " 0" << std::endl;
                    os << (offset_[Stench] + p) << " 0" << std::endl;
                }
            }
        }
        os << query << " 0" << std::endl;
    }

    void theory(std::ostream &os, const char *sensed_info, int query) const {
        header(os, 1 + ncls_extra_);
        cls_for_base_theory(os);
        other_cls(os, sensed_info, query);
    }

    bool make_inference(const char *sensed_info, int query) const {
        std::fstream of("theory.cnf", std::fstream::out);
        theory(of, sensed_info, query);
        of.close();
        return call_sat_solver();
    }

    bool call_sat_solver() const {
        //std::cout << "calling SAT-solver..." << std::flush;
        int rv = system(CMD_FOR_SAT_SOLVER);
        //std::cout << " done (rv=" << rv << ")" << std::endl;
        return rv == 0; // if rv == 0, theory.cnf is satisfiable!
    }

  public:
    exact_inference_t(int nrows, int ncols) : nrows_(nrows), ncols_(ncols) {
        assert(nrows_ > 1);
        assert(ncols_ > 1);
        ncells_ = nrows_ * ncols_;
        offset_[0] = 1;
        for( int i = 1; i < 4; ++i ) {
            offset_[i] = offset_[i-1] + ncells_;
        }
    }
    virtual ~exact_inference_t() { }

    bool is_world_explored(const char *sensed_info, bool verbose = false) const {
        std::vector<int> unvisited_cells_at_fringe;
        process_sensed_information(sensed_info, unvisited_cells_at_fringe);

        // make the inference
        for( unsigned i = 0; i < unvisited_cells_at_fringe.size(); ++i ) {
            int p = unvisited_cells_at_fringe[i];
            //std::cout << "querying: Wumpus@(" << p/ncols_ << "," << p%ncols_ << ")" << std::endl;
            //bool q = !make_inference(sensed_info, offset_[Wumpus] + p);
            //std::cout << "querying: Pit@(" << p/ncols_ << "," << p%ncols_ << ")" << std::endl;
            //q = q && !make_inference(sensed_info, offset_[Pit] + p);
            //if( q ) return false;
            if( !make_inference(sensed_info, offset_[Wumpus] + p) && !make_inference(sensed_info, offset_[Pit] + p) ) {
                if( verbose ) {
                    std::cout << "Warning: cell (" << p / ncols_ << "," << p % ncols_ << ") was not visited but safe to do it!"
                              << std::endl;
                }
                return false;
            }
        }
        return true;
    }
};

#endif

