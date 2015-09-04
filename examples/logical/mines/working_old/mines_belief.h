#ifndef MINES_BELIEF_H
#define MINES_BELIEF_H

#include "belief.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <stdlib.h>

class mines_belief_t : public belief_t {
  protected:
    std::vector<cell_beam_t*> beams_;
    mutable std::vector<int> cells_with_mine_;
    mutable std::vector<int> cells_without_mine_;
    std::vector<bool> marked_cells_;

  public:
    mines_belief_t() : belief_t() {
        beams_.reserve(rows_ * cols_);
        for( int r = 0; r < rows_; ++r ) {
            for( int c = 0; c < cols_; ++c ) {
                beams_.push_back(new cell_beam_t(r, c, types_[r * cols_ + c]));
            }
        }
        cells_with_mine_.reserve(rows_ * cols_);
        cells_without_mine_.reserve(rows_ * cols_);
        marked_cells_ = std::vector<bool>(rows_ * cols_, false);
    }
    explicit mines_belief_t(const mines_belief_t &bel) : belief_t(bel) {
        beams_.reserve(rows_ * cols_);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            beams_.push_back(new cell_beam_t(*bel.beams_[p]));
        }
        cells_with_mine_ = bel.cells_with_mine_;
        cells_without_mine_ = bel.cells_without_mine_;
        marked_cells_ = bel.marked_cells_;
    }
    virtual ~mines_belief_t() {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            delete beams_[p];
        }
    }

    static void initialize(int rows, int cols) {
        cell_beam_t::initialize();
        belief_t::initialize(rows, cols, belief_t::octile_neighbourhood);
    }

    void mark_cell(int cell) {
        marked_cells_[cell] = true;
    }
    bool marked_cell(int cell) const {
        return marked_cells_[cell];
    }

    std::vector<int>& cells_with_mine() const {
        return cells_with_mine_;
    }

    std::vector<int>& cells_without_mine() const {
        return cells_without_mine_;
    }

    virtual bool inconsistent() const {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            if( beams_[p]->empty() ) return true;
        }
        return false;
    }

    virtual void clear() {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            beams_[p]->clear();
        }
    }

    virtual void set_as_unknown() {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            beams_[p]->set_as_unknown(belief_t::octile_neighbourhood);
        }
        cells_with_mine_.clear();
        cells_without_mine_.clear();
        marked_cells_ = std::vector<bool>(rows_ * cols_, false);
    }

    void mine_filter(int cell, int nobjs) {
        filter(beams_, cell, nobjs);
        mine_ac3(cell);
    }

    virtual const mines_belief_t& operator=(const belief_t &bel) {
        static_cast<belief_t&>(*this) = bel;
        const mines_belief_t &mbel = static_cast<const mines_belief_t&>(bel);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            *beams_[p] = *mbel.beams_[p];
        }
        cells_with_mine_ = mbel.cells_with_mine_;
        cells_without_mine_ = mbel.cells_without_mine_;
        marked_cells_ = mbel.marked_cells_;
        return *this;
    }

    virtual bool operator==(const belief_t &bel) const {
        if( static_cast<const belief_t&>(*this) != bel ) return false;
        const mines_belief_t &mbel = static_cast<const mines_belief_t&>(bel);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            if( *beams_[p] != *mbel.beams_[p] ) return false;
        }
        return true;
    }
    virtual bool operator!=(const belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        for( int r = 0; r < rows_; ++r ) {
            for( int c = 0; c < cols_; ++c ) {
                int p = r * cols_ + c;
                os << "beam(" << c << "," << r << ")=" << *beams_[p] << std::endl;
            }
        }
    }

    virtual void mark_cell(std::vector<cell_beam_t*> &beams, int cell, bool hazard) {
        if( !marked_cells_[cell] ) {
            marked_cells_[cell] = true;
            if( hazard ) {
                cells_with_mine_.push_back(cell);
            } else {
                cells_without_mine_.push_back(cell);
            }
        }
    }

    void mine_ac3(int seed_beam, bool propagate = true) {
        ac3(beams_, seed_beam, propagate);
    }

    // Knowledge-query methods
    bool mine_at(int cell) const {
        return belief_t::hazard_at(beams_, cell);
    }
    bool no_mine_at(int cell) const {
        return belief_t::no_hazard_at(beams_, cell);
    }
    std::pair<int, int> num_surrounding_mines(int cell) const {
        return num_surrounding_objs(beams_, cell);
    }
    float mine_probability(int cell, float prior) const {
        return obj_probability(beams_, cell, prior);
    }
    bool virgin(int cell) const {
        return belief_t::virgin(beams_, cell);
    }

};

inline std::ostream& operator<<(std::ostream &os, const mines_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

