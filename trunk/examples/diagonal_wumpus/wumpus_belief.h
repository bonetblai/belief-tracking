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

#include <cassert>
#include <iostream>
#include <list>
#include <vector>


namespace Wumpus {

namespace Diagonal {

class wumpus_belief_t : public base_belief_t {
    static int dim_;
    static std::list<wumpus_belief_t*> beliefs_;

  protected:
    int pos_;
    int gold_pos_;
    bool have_gold_;
    bool dead_;

    arc_consistency_t wumpus_;

  public:
    wumpus_belief_t(int pos = 0, int gold_pos = 0)
      : base_belief_t(), pos_(pos), gold_pos_(gold_pos),
        have_gold_(false), dead_(false) {
        for( int i = 0; i < 3 * dim_ - 5; ++i ) {
             wumpus_.set_domain(i, new var_beam_t(1, 16));
        }
    }
    explicit wumpus_belief_t(const wumpus_belief_t &bel)
      : base_belief_t(bel), pos_(bel.pos_), gold_pos_(bel.gold_pos_),
        have_gold_(bel.have_gold_), dead_(bel.dead_) {
        for( int i = 0; i < 3 * dim_ - 5; ++i ) {
            wumpus_.set_domain(i, new var_beam_t(*bel.wumpus_.domain(i)));
        }
    }
#if 1
    wumpus_belief_t(wumpus_belief_t &&bel)
      : base_belief_t(bel), pos_(bel.pos_), gold_pos_(bel.gold_pos_),
        have_gold_(bel.have_gold_), dead_(bel.dead_) {
        for( int i = 0; i < 3 * dim_ - 5; ++i ) {
            wumpus_.set_domain(i, bel.wumpus_.domain(i));
            bel.wumpus_.set_domain(i, 0);
        }
    }
#endif
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

    static void initialize(int dim) {
        dim_ = dim;
        base_belief_t::initialize(dim_);
    }

    size_t hash() const { return 0; }

    int position() const { return pos_; }
    int gold_position() const { return gold_pos_; }
    bool have_gold() const { return have_gold_; }
    bool dead() const { return dead_; }
    void set_position(int pos) { pos_ = pos; }
    void set_gold_position(int pos) { gold_pos_ = pos; }

    bool consistent() const {
        for( int i = 0; i < 3 * dim_ - 5; ++i ) {
            if( wumpus_.domain(i)->empty() ) return false;
        }
        return true;
    }

    bool in_gold_cell() const { return pos_ == gold_pos_; }
    bool possible_gold_at(int cell) const { return gold_pos_ == cell; }

    void clear() {
        pos_ = 0;
        have_gold_ = false;
        dead_ = false;
        for( int i = 0; i < 3 * dim_ - 5; ++i ) {
            wumpus_.domain(i)->clear();
        }
    }

    bool consistent_particle(int row, int col, int p) const {
        valuation_t val(p);
        //std::cout << "particle: p=" << p << ", val=" << val << std::endl;
        if( row > col ) {
            return !val.n_wumpus_ && !val.w_wumpus_ && ((col > 0) || !val.s_wumpus_);
        } else if( row == col ) {
            if( col == 1 ) {
                return !val.s_wumpus_ && !val.w_wumpus_ &&
                       (val.n_wumpus_ || val.e_wumpus_) && (!val.n_wumpus_ || !val.e_wumpus_);
            } else if( col == dim_ - 1 ) {
                return !val.n_wumpus_ && !val.e_wumpus_ &&
                       (val.s_wumpus_ || val.w_wumpus_) && (!val.s_wumpus_ || !val.w_wumpus_);
            } else {
                return (val.n_wumpus_ || val.e_wumpus_) && (!val.n_wumpus_ || !val.e_wumpus_) &&
                       (val.s_wumpus_ || val.w_wumpus_) && (!val.s_wumpus_ || !val.w_wumpus_);
            }
        } else {
            return !val.e_wumpus_ && !val.s_wumpus_ && ((row > 0) || !val.w_wumpus_);
        }
    }

    void set_initial_configuration() {
        // Insert particles in beams. Two-pass method per beam: 1st pass calculate
        // number of particles in each beam, 2nd pass allocates space and insert
        // particles.
        for( int i = 0; i < 3 * dim_ - 5; ++i ) {
            var_beam_t &beam = *wumpus_.domain(i);
            beam.clear();
            position_t pos(dim_, i);
            int num_particles_in_beam = 0;
            for( int pass = 0; pass < 2; ++pass ) {
                if( pass == 1 ) beam.reserve(num_particles_in_beam);
                for( int p = 0; p < 16; ++p ) {
                    if( consistent_particle(pos.row_, pos.col_, p) ) {
                        //std::cout << "consistent!" << std::endl;
                        if( pass == 0 )
                            ++num_particles_in_beam;
                        else {
                            valuation_t val(p);
                            //std::cout << "particle: p=" << p << ", val=" << val << std::endl;
                            bool inserted = beam.push_back(p);
                            assert(inserted);
                        }
                    }
                }
            }
            //std::cout << "pos=(" << pos << "): size=" << beam.size() << std::endl;
            beam.set_initial_size(beam.size());
            assert(beam.initial_size() > 0);
        }
    }

    const wumpus_belief_t& operator=(const wumpus_belief_t &bel) {
        pos_ = bel.pos_;
        gold_pos_ = bel.gold_pos_;
        have_gold_ = bel.have_gold_;
        dead_ = bel.dead_;
        for( int i = 0; i < 3 * dim_ - 5; ++i ) {
            assert(wumpus_.domain(i) != 0);
            assert(bel.wumpus_.domain(i) != 0);
            *wumpus_.domain(i) = *bel.wumpus_.domain(i);
        }
        return *this;
    }

    virtual bool operator==(const wumpus_belief_t &bel) const {
        for( int i = 0; i < 3 * dim_ - 5; ++i ) {
            assert(wumpus_.domain(i) != 0);
            assert(bel.wumpus_.domain(i) != 0);
            if( *wumpus_.domain(i) != *bel.wumpus_.domain(i) ) return false;
        }
        return (pos_ == bel.pos_) &&
               (gold_pos_ == bel.gold_pos_) &&
               (have_gold_ == bel.have_gold_) && (dead_ == bel.dead_);
    }
    virtual bool operator!=(const wumpus_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        os << "pos=(" << (pos_ % dim_) << "," << (pos_ / dim_) << ")"
           << ", gold_pos=(" << (gold_pos_ % dim_) << "," << (gold_pos_ / dim_) << ")"
           << ", have-gold=" << (have_gold_ ? "true" : "false")
           << ", dead=" << (dead_ ? "true" : "false")
           << std::endl;
#if 1
        for( int i = 0; i < 3 * dim_ - 5; ++i ) {
            os << "beam(" << i << ")=" << *wumpus_.domain(i) << std::endl;
        }
#endif
    }

    int target_cell(int action) const {
        return ::target_cell(pos_, 0, action, dim_, dim_, true);
    }

    bool applicable(int action) const {
        return applicable_for_position(action) &&
               applicable_for_gold(action) &&
               applicable_for_wumpus(action);
    }

    bool possible_obs(int action, int obs) const {
        if( pos_ == OutsideCave ) {
            return obs == 0;
        } else {
            if( obs < Fell ) {
                return possible_obs_for_gold(action, obs) &&
                       possible_obs_for_wumpus(action, obs);
            } else if ( obs == Fell ) {
                return false;
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
        progress_wumpus(action);
    }

    void filter(int action, int obs) {
        if( pos_ != OutsideCave ) {
            if( obs < Fell ) {
                filter_gold(action, obs);
                filter_wumpus(action, obs);
            } else if( obs == Fell ) {
                assert(0);
            } else if( obs == Eaten ) {
                dead_ = true;
                filter_and_propagate_killed_at(pos_);
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
    std::pair<int, int> num_surrounding_wumpus(int row, int col, const var_beam_t &beam) const {
        int min = 4, max = 0;
        for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            valuation_t val(*it);
            int count = (val.n_wumpus_ ? 1 : 0) + (val.e_wumpus_ ? 1 : 0) +
                        (val.s_wumpus_ ? 1 : 0) + (val.w_wumpus_ ? 1 : 0);
            min = count < min ? count : min;
            max = count > max ? count : max;
        }
        return std::make_pair(min, max);
    }
    bool wumpus_at(int row, int col, const var_beam_t &beam, int from, bool sign = true) const {
        for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            valuation_t val(*it);
            if( from == 0 ) { // 0 == North
                if( sign && !val.n_wumpus_ ) return false;
                if( !sign && val.n_wumpus_ ) return false;
            } else if( from == 1 ) { // 1 == East
                if( sign && !val.e_wumpus_ ) return false;
                if( !sign && val.e_wumpus_ ) return false;
            } else if( from == 2 ) { // 2 == South
                if( sign && !val.s_wumpus_ ) return false;
                if( !sign && val.s_wumpus_ ) return false;
            } else { // 3 == West
                if( sign && !val.w_wumpus_ ) return false;
                if( !sign && val.w_wumpus_ ) return false;
            }
        }
        return !beam.empty();
    }
    bool wumpus_at(int cell) const {
      int row = cell / dim_, col = cell % dim_;
      if( (col > 0) && (1 + col == row) ) {
          return wumpus_at(row, col, *wumpus_.domain(col - 1), 0); // 0 == North
      } else if( (row > 0) && (col == 1 + row) ) {
          return wumpus_at(row, col, *wumpus_.domain(row - 1), 1); // 1 == East
      } else {
          return false;
      }
    }
    bool no_wumpus_at(int cell) const {
      int row = cell / dim_, col = cell % dim_;
      if( (col > 0) && (1 + col == row) ) {
          return wumpus_at(row, col, *wumpus_.domain(col - 1), 0, false); // 0 == North
      } else if( (row > 0) && (col == 1 + row) ) {
          return wumpus_at(row, col, *wumpus_.domain(row - 1), 1, false); // 1 == East
      } else {
          return true;
      }
    }
    std::pair<int, int> num_surrounding_wumpus(int cell) const {
      int row = cell / dim_, col = cell % dim_;
      if( (col > 0) && (col == row) ) { // main diagonal
          return num_surrounding_wumpus(row, col, *wumpus_.domain(col - 1));
      } else if( 2 + col == row ) { // above diagonal
          return num_surrounding_wumpus(row, col, *wumpus_.domain(dim_ - 1 + col));
      } else if( col == 2 + row ) { // below diagonal
          return num_surrounding_wumpus(row, col, *wumpus_.domain(2 * dim_ - 3 + row));
      } else {
          return std::make_pair(0, 0);
      }
    }
    bool hazard_at(int cell) const { return wumpus_at(cell); }
    bool no_hazard_at(int cell) const { return no_wumpus_at(cell); }

  private:
    bool applicable_for_position(int action) const {
        if( dead_ || (pos_ == OutsideCave) ) { // CHECK: removed || (action == ActionNoop) ) {
            return false;
        } else if( action == ActionShoot ) {
            return false;
        } else if( action == ActionExit ) {
            return pos_ == 0;
        } else if( (action == ActionMoveNorth) || (action == ActionMoveEast) ||
                   (action == ActionMoveSouth) || (action == ActionMoveWest) ) {
            //std::cout << "tcell=" << target_cell(action) << ", cell=" << pos_ << std::endl;
            return target_cell(action) != pos_;
        } else {
            return true;
        }
    }
    void progress_position(int action) {
        if( action == ActionExit ) {
            pos_ = OutsideCave;
        } else if( (action == ActionMoveNorth) || (action == ActionMoveEast) ||
                   (action == ActionMoveSouth) || (action == ActionMoveWest) ) {
            pos_ = target_cell(action);
            //std::cout << "new pos after action=" << action << " is " << pos_ << std::endl;
        }
    }

    bool applicable_for_gold(int action) const {
        //std::cout << "action=" << action << ", grab=" << ActionGrab << ", gold-pos=" << gold_pos_ << std::endl;
        return (action != ActionGrab) || in_gold_cell();
    }
    void progress_gold(int action) {
        if( action == ActionGrab ) {
            assert(!have_gold());
            assert(in_gold_cell());
            have_gold_ = true;
        }
    }
    bool possible_obs_for_gold(int action, int obs) const {
        if( obs & Glitter ) {
            return !have_gold() && (gold_pos_ == pos_);
        } else {
            return have_gold() || (gold_pos_ != pos_);
        }
    }
    void filter_gold(int action, int obs) {
        // nothing to do
    }

    bool applicable_for_wumpus(int action) const {
        return true;
    }
    void progress_wumpus(int action) {
        // nothing to do
    }
    bool possible_obs_for_wumpus(int action, int obs) const {
        std::pair<int, int> count = num_surrounding_wumpus(pos_);
        if( obs & Stench ) {
            return count.second > 0;
        } else {
            return count.first == 0;
        }
    }
    void filter_wumpus(int action, int obs) {
        if( obs & Stench ) {
            filter_and_propagate(pos_, 1, true);
        } else {
            filter_and_propagate(pos_, 0, false);
        }
    }

    void filter_and_propagate(int cell, int nobjs, bool at_least) {
        // locate beam on which to make the filtering
        int row = cell / dim_, col = cell % dim_;
        int beam_index = 0;
        if( 2 + col == row ) { // above-diagonal beam
            beam_index = col + dim_ - 1;
        } else if( (col > 0) && (col == row) ) { // diagonal beam
            beam_index = col - 1;
        } else if( col == 2 + row ) { // below-diagonal beam
            beam_index = col + 2 * dim_ - 5;
        } else {
            // nothing to do; return
            return;
        }

        // Remove particles not consistent with condition:
        //     at_least: each valuation must have at least nobjs
        //     !at_least: each valuation must have at most nobjs
        var_beam_t &beam = *wumpus_.domain(beam_index);
        static std::vector<int> indices_to_erase;
        indices_to_erase.clear();
        indices_to_erase.reserve(beam.size());
        for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            valuation_t val(*it);
            int n = val.n_wumpus_ ? 1 : 0;
            n += val.e_wumpus_ ? 1 : 0;
            n += val.s_wumpus_ ? 1 : 0;
            n += val.w_wumpus_ ? 1 : 0;
            if( (at_least && (n < nobjs)) || (!at_least && (n > nobjs)) ) {
                indices_to_erase.push_back(it.index());
            }
        }
        beam.erase_ordered_indices(indices_to_erase);

        // propagate filtering to other beams
        std::vector<int> revised_cells;
        wumpus_.add_to_worklist(beam_index);
        wumpus_.ac3(revised_cells);
    }

    void filter_and_propagate_killed_at(int cell) {
        // locate beam on which to make the filtering
        int row = cell / dim_, col = cell % dim_;
        assert((1 + col == row) || (col == 1 + row));
        assert((row > 0) && (col > 0));
        int beam_index = 0;
        if( 1 + col == row ) { // above-diagonal wumpus, use diagonal beam
            beam_index = col - 1;
        } else { // below-diagonal wumpus, use below-diagonal beam
            beam_index = col + 2 * dim_ - 5;
        }

        // Remove particles that do not contain a wumpus at North
        var_beam_t &beam = *wumpus_.domain(beam_index);
        static std::vector<int> indices_to_erase;
        indices_to_erase.clear();
        indices_to_erase.reserve(beam.size());
        for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            valuation_t val(*it);
            if( !val.n_wumpus_ ) {
                indices_to_erase.push_back(it.index());
            }
        }
        beam.erase_ordered_indices(indices_to_erase);

        // propagate filtering to other beams
        std::vector<int> revised_cells;
        wumpus_.add_to_worklist(beam_index);
        wumpus_.ac3(revised_cells);
    }
};

int wumpus_belief_t::dim_ = 0;
std::list<wumpus_belief_t*> wumpus_belief_t::beliefs_;

};

};

inline std::ostream& operator<<(std::ostream &os, const Wumpus::Diagonal::wumpus_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

