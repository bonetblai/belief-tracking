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

#ifndef MOVING_WUMPUS_BELIEF_H
#define MOVING_WUMPUS_BELIEF_H

#include "grid_belief.h"
#include "defs.h"
#include <grid_var_beam.h>

#include <stdlib.h>
#include <cassert>
#include <iostream>
#include <list>
#include <vector>

class moving_wumpus_belief_t : public grid_belief_t {
    static int nrows_;
    static int ncols_;
    static int npits_;
    static int nwumpus_;
    static std::list<moving_wumpus_belief_t*> beliefs_;

  protected:
    int pos_;
    int heading_;
    int narrows_;
    bool have_gold_;
    bool dead_;

    grid_var_beam_t gold_;
    grid_arc_consistency_t pits_;
    grid_var_beam_t wumpus_;

  public:
    moving_wumpus_belief_t(int pos = 0, int heading = 0)
      : grid_belief_t(), pos_(pos), heading_(heading), narrows_(0),
        have_gold_(false), dead_(false),
        gold_(1), wumpus_(nwumpus_) {
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                pits_.set_domain(cell, new cell_beam_t(r, c, types_[cell]));
            }
        }
        if( nwumpus_ > 1 ) {
            std::cout << "error: number of wumpus should be at most one!" << std::endl;
            exit(-1);
        }
    }
    explicit moving_wumpus_belief_t(const moving_wumpus_belief_t &bel)
      : grid_belief_t(bel), pos_(bel.pos_), heading_(bel.heading_), narrows_(bel.narrows_),
        have_gold_(bel.have_gold_), dead_(bel.dead_),
        gold_(bel.gold_), wumpus_(bel.wumpus_) {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.set_domain(cell, new cell_beam_t(*bel.pits_.domain(cell)));
        }
    }
#if 0
    moving_wumpus_belief_t(moving_wumpus_belief_t &&bel)
      : grid_belief_t(bel), pos_(bel.pos_), heading_(bel.heading_), narrows_(bel.narrows_),
        have_gold_(bel.have_gold_), dead_(bel.dead_),
        gold_(std::move(bel.gold_)), wumpus_(std::move(bel.wumpus_)) {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.set_domain(cell, bel.pits_.domain(cell));
            bel.pits_.set_domain(cell, 0);
        }
    }
#endif
    virtual ~moving_wumpus_belief_t() { }

    static moving_wumpus_belief_t* allocate() {
        if( beliefs_.empty() ) {
            return new moving_wumpus_belief_t;
        } else {
            moving_wumpus_belief_t *belief = beliefs_.front();
            beliefs_.pop_front();
            assert(belief != 0);
            belief->clear();
            return belief;
        }
    }
    static void deallocate(moving_wumpus_belief_t *belief) {
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
        grid_belief_t::initialize(nrows_, ncols_, grid_belief_t::manhattan_neighbourhood);
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
        }
        return wumpus_.consistent() && (have_gold() || gold_.consistent());
    }

    bool in_gold_cell() const { return gold_.known() && (*gold_.begin() == pos_); }
    bool possible_gold_at(int cell) const { return gold_.contains(cell); }

    void clear() {
        pos_ = 0;
        heading_ = 0;
        have_gold_ = false;
        dead_ = false;
        gold_.clear();
        wumpus_.clear();
        for( int cell = 0; cell < rows_ * cols_; ++cell ) {
            pits_.domain(cell)->clear();
        }
    }

    void set_initial_configuration() {
        gold_.set_initial_configuration();
        wumpus_.set_initial_configuration();
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.domain(cell)->set_initial_configuration(grid_belief_t::manhattan_neighbourhood);
        }
    }

    const moving_wumpus_belief_t& operator=(const moving_wumpus_belief_t &bel) {
        pos_ = bel.pos_;
        heading_ = bel.heading_;
        have_gold_ = bel.have_gold_;
        dead_ = bel.dead_;
        gold_ = bel.gold_;
        wumpus_ = bel.wumpus_;
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            assert(pits_.domain(cell) != 0);
            *pits_.domain(cell) = *bel.pits_.domain(cell);
        }
        return *this;
    }

    bool operator==(const moving_wumpus_belief_t &bel) const {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            if( *pits_.domain(cell) != *bel.pits_.domain(cell) ) return false;
        }
        return (pos_ == bel.pos_) && (heading_ == bel.heading_) &&
               (have_gold_ == bel.have_gold_) && (dead_ == bel.dead_) &&
               (gold_ == bel.gold_) && (wumpus_ == bel.wumpus_);
    }
    bool operator!=(const moving_wumpus_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        os << "pos=(" << (pos_ % ncols_)
           << "," << (pos_ / ncols_)
           << "," << heading_name(heading_) << ")"
           << ", have-gold=" << (have_gold_ ? "true" : "false")
           << ", dead=" << (dead_ ? "true" : "false")
           << std::endl;
        os << "gold=" << gold_ << std::endl;
        os << "wumpus=" << wumpus_ << std::endl;
#if 1
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                os << "pbeam(" << c << "," << r << ")=" << *pits_.domain(cell) << std::endl;
            }
        }
#endif
    }

    int target_cell(int action) const {
        return ::target_cell(pos_, heading_, action, nrows_, ncols_, false);
    }
    int target_heading(int action) const {
        return ::target_heading(heading_, action);
    }

    bool applicable(int action) const {
        return applicable_for_position(action) &&
               applicable_for_gold(action) &&
               applicable_for_pits(action) &&
               applicable_for_wumpus(action);
    }

    bool possible_obs(int action, int obs) const {
        assert((obs >= 0) && (obs < 105));
        if( pos_ == OutsideCave ) {
            return obs == 0;
        } else if( obs == 104 ) { // Fell into a pit
            return !no_pit_at(pos_);
        } else {
            return possible_obs_for_gold(action, obs) &&
                   possible_obs_for_pits(action, obs) &&
                   possible_obs_for_wumpus(action, obs);
        }
    }

    void progress(int action) {
        assert(applicable(action));
        progress_position(action);
        progress_gold(action);
        progress_pits(action);
        progress_wumpus(action);
    }

    bool is_dead_obs(int obs) const {
        // wumpus at cell (52 = 4 * 13) or fell into pit (104)
        return (obs == 52) || (obs == 104);
    }

    void filter(int action, int obs) {
        grid_var_beam_t before(wumpus_);
        if( is_dead_obs(obs) ) {
            dead_ = true;
        } else {
            if( pos_ != OutsideCave ) {
                filter_gold(action, obs);
                filter_pits(action, obs);
                filter_wumpus(action, obs);
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
    bool wumpus_at(int cell) const { return wumpus_.necessary_obj_at(cell); }
    bool hazard_at(int cell) const { return pit_at(cell) || wumpus_at(cell); }
    bool no_pit_at(int cell) const { return pits_.domain(cell)->no_obj_at(); }
    bool no_wumpus_at(int cell) const { return wumpus_.necessary_no_obj_at(cell); }
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
            gold_.filter_obj_at(pos_);
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
        //if( action < ActionGrab ) {
        if( action == ActionMoveForward ) {
            // this is a move action, the wumpuses move non-det in the grid
            wumpus_.move_non_det();
        }
    }
    bool possible_obs_for_wumpus(int action, int obs) const {
        assert((obs >= 0) && (obs < 104));
        int wumpus_obs = obs >> 2;
        if( wumpus_obs == 0 ) {
            return wumpus_.possible_no_obj_adjacent_to(pos_, grid_var_beam_t::octile_neighbourhood, 2);
        } else {
            --wumpus_obs;
            int drow = (wumpus_obs / 5) - 2, dcol = (wumpus_obs % 5) - 2;
            int row = pos_ / ncols_, col = pos_ % ncols_;
            int wrow = row + drow, wcol = col + dcol;
            if( (wrow >= 0) && (wrow < nrows_) && (wcol >= 0) && (wcol < ncols_) ) {
                int wpos = wrow * ncols_ + wcol;
                return wumpus_.possible_obj_at(wpos);
            } else {
                return false;
            }
        }
    }
    void filter_wumpus(int action, int obs) {
        assert((obs >= 0) && (obs < 104));
        int wumpus_obs = obs >> 2;
        if( wumpus_obs == 0 ) {
            return wumpus_.filter_no_obj_adjacent_to(pos_, grid_var_beam_t::octile_neighbourhood, 2);
        } else {
            --wumpus_obs;
            int drow = (wumpus_obs / 5) - 2, dcol = (wumpus_obs % 5) - 2;
            int row = pos_ / ncols_, col = pos_ % ncols_;
            int wrow = row + drow, wcol = col + dcol;
            assert((wrow >= 0) && (wrow < nrows_) && (wcol >= 0) && (wcol < ncols_));
            int wpos = wrow * ncols_ + wcol;
            wumpus_.filter_obj_at(wpos);
        }
    }

    void pit_filter(int cell, int nobjs, bool at_least) {
        std::vector<int> revised_cells;
        pits_.domain(cell)->filter(nobjs, at_least);
        pits_.add_to_worklist(cell);
        pits_.ac3(revised_cells);
    }
};

int moving_wumpus_belief_t::nrows_ = 0;
int moving_wumpus_belief_t::ncols_ = 0;
int moving_wumpus_belief_t::npits_ = 0;
int moving_wumpus_belief_t::nwumpus_ = 0;
std::list<moving_wumpus_belief_t*> moving_wumpus_belief_t::beliefs_;

inline std::ostream& operator<<(std::ostream &os, const moving_wumpus_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

