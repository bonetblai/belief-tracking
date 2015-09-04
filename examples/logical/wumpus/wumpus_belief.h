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

#ifndef WUMPUS_BELIEF_H
#define WUMPUS_BELIEF_H

#include "base_belief.h"
#include "defs.h"
#include <grid_var_beam.h>

#include <cassert>
#include <iostream>
#include <list>
#include <vector>


namespace Wumpus {

class wumpus_belief_t : public base_belief_t {
    static int nrows_;
    static int ncols_;
    static int npits_;
    static int nwumpus_;
    static std::list<wumpus_belief_t*> beliefs_;

  protected:
    int pos_;
    int heading_;
    int narrows_;
    bool have_gold_;
    bool dead_;

    grid_var_beam_t gold_;
    arc_consistency_t pits_;
    arc_consistency_t wumpus_;

  public:
    wumpus_belief_t(int pos = 0, int heading = 0)
      : base_belief_t(), pos_(pos), heading_(heading), narrows_(0),
        have_gold_(false), dead_(false), gold_(1) {
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                pits_.set_domain(cell, new cell_beam_t(r, c, types_[cell]));
                wumpus_.set_domain(cell, new cell_beam_t(r, c, types_[cell]));
            }
        }
    }
    explicit wumpus_belief_t(const wumpus_belief_t &bel)
      : base_belief_t(bel), pos_(bel.pos_), heading_(bel.heading_), narrows_(bel.narrows_),
        have_gold_(bel.have_gold_), dead_(bel.dead_), gold_(bel.gold_) {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.set_domain(cell, new cell_beam_t(*bel.pits_.domain(cell)));
            wumpus_.set_domain(cell, new cell_beam_t(*bel.wumpus_.domain(cell)));
        }
    }
    wumpus_belief_t(wumpus_belief_t &&bel)
      : base_belief_t(bel), pos_(bel.pos_), heading_(bel.heading_), narrows_(bel.narrows_),
        have_gold_(bel.have_gold_), dead_(bel.dead_), gold_(bel.gold_) {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.set_domain(cell, bel.pits_.domain(cell));
            bel.pits_.set_domain(cell, 0);
            wumpus_.set_domain(cell, bel.wumpus_.domain(cell));
            bel.wumpus_.set_domain(cell, 0);
        }
    }
    virtual ~wumpus_belief_t() { }

    static wumpus_belief_t* allocate() {
        if( beliefs_.empty() ) {
            return new wumpus_belief_t;
        } else {
            wumpus_belief_t *belief = beliefs_.front();
            beliefs_.pop_front();
            assert(belief != 0);
            belief->clear();
            return belief;
        }
    }
    static void deallocate(wumpus_belief_t *belief) {
        if( belief != 0 ) {
            beliefs_.push_front(belief);
        }
    }

    static void initialize(int nrows, int ncols, int npits, int nwumpus) {
        nrows_ = nrows;
        ncols_ = ncols;
        npits_ = npits;
        nwumpus_ = nwumpus;
        cell_beam_t::initialize();
        grid_var_beam_t::initialize(nrows_, ncols_);
        base_belief_t::initialize(nrows_, ncols_, base_belief_t::manhattan_neighbourhood);
    }

    size_t hash() const { return 0; }

    int position() const { return pos_; }
    int heading() const { return heading_; }
    bool have_gold() const { return have_gold_; }
    bool dead() const { return dead_; }
    void set_position(int pos) { pos_ = pos; }
    void set_heading(int heading) { heading_ = heading; }

    bool consistent() const {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            if( pits_.domain(cell)->empty() ) return false;
            if( wumpus_.domain(cell)->empty() ) return false;
        }
        return have_gold() || gold_.consistent();
    }

    bool in_gold_cell() const { return gold_.known() && (*gold_.begin() == pos_); }
    bool possible_gold_at(int cell) const { return gold_.contains(cell); }

    void clear() {
        pos_ = 0;
        heading_ = 0;
        have_gold_ = false;
        dead_ = false;
        gold_.clear();
        for( int cell = 0; cell < rows_ * cols_; ++cell ) {
            pits_.domain(cell)->clear();
            wumpus_.domain(cell)->clear();
        }
    }

    void set_initial_configuration(bool diagonal) {
        gold_.set_initial_configuration();
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.domain(cell)->set_initial_configuration(base_belief_t::manhattan_neighbourhood);
            wumpus_.domain(cell)->set_initial_configuration(base_belief_t::manhattan_neighbourhood);
        }

        // If diagonal version of wumpus, filter beams with the 
        // a priori knowledge. Then, run arc consistency.
        if( diagonal ) {
            // No pits and wumpus can only be above or below the
            // main diagonal.
            for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
                int row = cell / ncols_, col = cell % ncols_;
                pits_.domain(cell)->remove_obj_at_cell();
                if( ((col == 0) || (1 + col != row)) && ((row == 0) || (col != 1 + row)) ) {
                    wumpus_.domain(cell)->remove_obj_at_cell();
                }
            }

            // Given wumpus cannot be at its two different possible
            // locations, and must be in one of them
            std::vector<int> indices_to_erase;
            for( int row = 2; row < nrows_; ++row ) {
                int col = row, cell = row * ncols_ + col;
                cell_beam_t &beam = *wumpus_.domain(cell);
                indices_to_erase.clear();
                indices_to_erase.reserve(beam.size());
                for( cell_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
                    int p = *it;
                    if( ((p >> 1) & 0x1) == ((p >> 3) & 0x1) ) {
                        indices_to_erase.push_back(it.index());
                    }
                }
                beam.erase_ordered_indices(indices_to_erase);
            }

            // run arc consistency
            std::vector<int> revised_cells;
            pits_.add_all_edges_to_worklist();
            pits_.ac3(revised_cells);

            revised_cells.clear();
            wumpus_.add_all_edges_to_worklist();
            wumpus_.ac3(revised_cells);
        }
    }

    const wumpus_belief_t& operator=(const wumpus_belief_t &bel) {
        pos_ = bel.pos_;
        heading_ = bel.heading_;
        have_gold_ = bel.have_gold_;
        dead_ = bel.dead_;
        gold_ = bel.gold_;
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            assert(pits_.domain(cell) != 0);
            assert(wumpus_.domain(cell) != 0);
            *pits_.domain(cell) = *bel.pits_.domain(cell);
            *wumpus_.domain(cell) = *bel.wumpus_.domain(cell);
        }
        return *this;
    }

    virtual bool operator==(const wumpus_belief_t &bel) const {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            if( *pits_.domain(cell) != *bel.pits_.domain(cell) ) return false;
            if( *wumpus_.domain(cell) != *bel.wumpus_.domain(cell) ) return false;
        }
        return (pos_ == bel.pos_) && (heading_ == bel.heading_) &&
               (have_gold_ == bel.have_gold_) && (dead_ == bel.dead_) &&
               (gold_ == bel.gold_);
    }
    virtual bool operator!=(const wumpus_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        os << "pos=(" << (pos_ % ncols_)
           << "," << (pos_ / ncols_)
           << "," << heading_name(heading_) << ")"
           << ", have-gold=" << (have_gold_ ? "true" : "false")
           << ", dead=" << (dead_ ? "true" : "false")
           << std::endl;
#if 1
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                //os << "pbeam(" << c << "," << r << ")=" << *pits_.domain(cell) << std::endl;
                os << "wbeam(" << c << "," << r << ")=" << *wumpus_.domain(cell) << std::endl;
            }
        }
#endif
    }

    int target_cell(int action) const {
        return Wumpus::target_cell(pos_, heading_, action, nrows_, ncols_, false);
    }
    int target_heading(int action) const {
        return Wumpus::target_heading(heading_, action);
    }

    bool applicable(int action) const {
        return applicable_for_position(action) &&
               applicable_for_gold(action) &&
               applicable_for_pits(action) &&
               applicable_for_wumpus(action);
    }

    bool possible_obs(int action, int obs) const {
        if( pos_ == OutsideCave ) {
            return obs == 0;
        } else {
            if( obs < Fell ) {
                return possible_obs_for_gold(action, obs) &&
                       possible_obs_for_pits(action, obs) &&
                       possible_obs_for_wumpus(action, obs);
            } else if ( obs == Fell ) {
                return !no_pit_at(pos_);
            } else if( obs == Eaten ) {
                return !no_wumpus_at(pos_);
            } else {
                return false;
            }
        }
    }

    void progress(int action) {
        assert(applicable(action));
        progress_position(action);
        progress_gold(action);
        progress_pits(action);
        progress_wumpus(action);
    }

    void filter(int action, int obs) {
        if( pos_ != OutsideCave ) {
            if( obs < Fell ) {
                if( pos_ != OutsideCave ) {
                    filter_gold(action, obs);
                    filter_pits(action, obs);
                    filter_wumpus(action, obs);
                }
            } else if( obs == Fell ) {
                dead_ = true;
                pit_filter(pos_, 9, false);
            } else if( obs == Eaten ) {
                dead_ = true;
                wumpus_filter(pos_, 9, false);
            } else {
                assert(0);
            }
        }
        assert(consistent());
    }

    void progress_and_filter(int action, int obs) {
        progress(action);
        filter(action, obs);
    }

    // Knowledge-query methods
    bool pit_at(int cell) const { return pits_.domain(cell)->obj_at(); }
    bool wumpus_at(int cell) const { return wumpus_.domain(cell)->obj_at(); }
    bool hazard_at(int cell) const { return pit_at(cell) || wumpus_at(cell); }
    bool no_pit_at(int cell) const { return pits_.domain(cell)->no_obj_at(); }
    bool no_wumpus_at(int cell) const { return wumpus_.domain(cell)->no_obj_at(); }
    bool no_hazard_at(int cell) const { return no_pit_at(cell) && no_wumpus_at(cell); }

  private:
    bool applicable_for_position(int action) const {
        if( dead_ || (pos_ == OutsideCave) || (action == ActionNoop) ) {
            return false;
        } else if( action == ActionShoot ) {
            return narrows_ > 0;
        } else if( action == ActionExit ) {
            return pos_ == 0;
        } else if( action == ActionMoveForward ) {
            return target_cell(action) != pos_;
        } else {
            return true;
        }
    }
    void progress_position(int action) {
        if( action == ActionExit ) {
            pos_ = OutsideCave;
        } else if( action == ActionMoveForward ) {
            pos_ = target_cell(action);
        } else if( (action == ActionTurnLeft) || (action == ActionTurnRight) ) {
            heading_ = target_heading(action);
        }
    }

    bool applicable_for_gold(int action) const {
        return (action != ActionGrab) || in_gold_cell();
    }
    void progress_gold(int action) {
        if( action == ActionGrab ) {
            assert(!have_gold());
            assert(in_gold_cell());
            gold_.clear();
            have_gold_ = true;
        }
    }
    bool possible_obs_for_gold(int action, int obs) const {
        if( obs & Glitter ) {
            return gold_.possible_obj_at(pos_);
        } else {
            return gold_.empty() || gold_.possible_no_obj_at(pos_);
        }
    }
    void filter_gold(int action, int obs) {
        if( obs & Glitter ) {
            gold_.clear();
            gold_.insert(pos_);
        } else {
            gold_.filter_no_obj_at(pos_);
        }
    }

    bool applicable_for_pits(int action) const {
        return true;
    }
    void progress_pits(int action) {
        // nothing to do
    }
    bool possible_obs_for_pits(int action, int obs) const {
        std::pair<int, int> min_max_pits = pits_.domain(pos_)->num_surrounding_objs();    
        if( obs & Breeze ) {
            return min_max_pits.second > 0;
        } else {
            return min_max_pits.first == 0;
        }
    }
    void filter_pits(int action, int obs) {
        if( obs & Breeze ) {
            pit_filter(pos_, 1, true);
        } else {
            pit_filter(pos_, 0, false);
        }
    }

    bool applicable_for_wumpus(int action) const {
        return true;
    }
    void progress_wumpus(int action) {
        // nothing to do
    }
    bool possible_obs_for_wumpus(int action, int obs) const {
        std::pair<int, int> min_max_wumpus = wumpus_.domain(pos_)->num_surrounding_objs();    
        if( obs & Stench ) {
            return min_max_wumpus.second > 0;
        } else {
            return min_max_wumpus.first == 0;
        }
    }
    void filter_wumpus(int action, int obs) {
        if( obs & Stench ) {
            wumpus_filter(pos_, 1, true);
        } else {
            wumpus_filter(pos_, 0, false);
        }
    }

    void pit_filter(int cell, int nobjs, bool at_least) {
        std::vector<int> revised_cells;
        pits_.domain(cell)->filter(nobjs, at_least);
        pits_.add_to_worklist(cell);
        pits_.ac3(revised_cells);
    }

    void wumpus_filter(int cell, int nobjs, bool at_least) {
        std::vector<int> revised_cells;
        wumpus_.domain(cell)->filter(nobjs, at_least);
        wumpus_.add_to_worklist(cell);
        wumpus_.ac3(revised_cells);
    }
};

int wumpus_belief_t::nrows_ = 0;
int wumpus_belief_t::ncols_ = 0;
int wumpus_belief_t::npits_ = 0;
int wumpus_belief_t::nwumpus_ = 0;
std::list<wumpus_belief_t*> wumpus_belief_t::beliefs_;

};

inline std::ostream& operator<<(std::ostream &os, const Wumpus::wumpus_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

