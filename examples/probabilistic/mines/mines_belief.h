#ifndef MINES_BELIEF_H
#define MINES_BELIEF_H

#include "mines_grid_belief.h"

#include <cassert>
#include <iostream>
#include <list>
#include <vector>

class mines_belief_t {
    static int nrows_;
    static int ncols_;
    static int ncells_;
    static int nmines_;
    static std::list<mines_belief_t*> beliefs_;

  protected:
    int nflags_;
    int ncovered_;

    grid_belief_t *belief_;

    mutable std::vector<int> cells_with_mine_;
    mutable std::vector<int> cells_without_mine_;
    std::vector<bool> marked_cells_;

    std::vector<bool> flag_;
    std::vector<bool> uncovered_;

  public:
    mines_belief_t()
      : nflags_(0), ncovered_(ncells_), belief_(0) {
        belief_ = new grid_belief_t;
        cells_with_mine_.reserve(ncells_);
        cells_without_mine_.reserve(ncells_);
        marked_cells_ = std::vector<bool>(ncells_, false);
        flag_ = std::vector<bool>(ncells_, false);
        uncovered_ = std::vector<bool>(ncells_, false);
    }
    explicit mines_belief_t(const mines_belief_t &bel)
      : nflags_(bel.nflags_), ncovered_(bel.ncovered_), belief_(0) {
        belief_ = new grid_belief_t(*bel.belief_);
        cells_with_mine_ = bel.cells_with_mine_;
        cells_without_mine_ = bel.cells_without_mine_;
        marked_cells_ = bel.marked_cells_;
        flag_ = bel.flag_;
        uncovered_ = bel.uncovered_;
    }
    mines_belief_t(mines_belief_t &&bel)
      : nflags_(bel.nflags_), ncovered_(bel.ncovered_), belief_(bel.belief_) {
        bel.belief_ = 0;
        cells_with_mine_ = std::move(bel.cells_with_mine_);
        cells_without_mine_ = std::move(bel.cells_without_mine_);
        marked_cells_ = std::move(bel.marked_cells_);
        flag_ = std::move(bel.flag_);
        uncovered_ = std::move(bel.uncovered_);
    }
    virtual ~mines_belief_t() {
        delete belief_;
    }

    static mines_belief_t* allocate() {
        if( beliefs_.empty() ) {
            return new mines_belief_t;
        } else {
            mines_belief_t *belief = beliefs_.front();
            beliefs_.pop_front();
            assert(belief != 0);
            belief->clear();
            return belief;
        }
    }
    static void deallocate(mines_belief_t *belief) {
        if( belief != 0 ) {
            beliefs_.push_front(belief);
        }
    }
    static void release_memory() {
        for( std::list<mines_belief_t*>::const_iterator it = beliefs_.begin(); it != beliefs_.end(); ++it ) {
            delete *it;
        }
        beliefs_.clear();
    }

    static void initialize(int nrows, int ncols, int nmines) {
        nrows_ = nrows;
        ncols_ = ncols;
        ncells_ = nrows_ * ncols_;
        nmines_ = nmines;
        cell_beam_t::initialize();
        grid_belief_t::initialize(nrows_, ncols_, grid_belief_t::octile_neighbourhood);
    }
    static void finalize() {
        release_memory();
        grid_belief_t::finalize();
        cell_beam_t::finalize();
    }

    static int nrows() { return nrows_; }
    static int ncols() { return ncols_; }
    static int ncells() { return ncells_; }

    size_t hash() const { return 0; }

    int nflags() const { return nflags_; }
    void set_nflags(int nflags) { nflags_ = nflags; }

    int ncovered() const { return ncovered_; }
    const std::vector<int>& cells_with_mine() const { return cells_with_mine_; }
    const std::vector<int>& cells_without_mine() const { return cells_without_mine_; }

    bool consistent() const {
        return belief_->consistent();
    }

    void clear() {
        belief_->clear();
        cells_with_mine_.clear();
        cells_without_mine_.clear();
        marked_cells_ = std::vector<bool>(ncells_, false);
        flag_ = std::vector<bool>(ncells_, false);
        uncovered_ = std::vector<bool>(ncells_, false);
    }

    void set_initial_configuration() {
        belief_->set_initial_configuration();
        cells_with_mine_.clear();
        cells_without_mine_.clear();
        marked_cells_ = std::vector<bool>(ncells_, false);
        flag_ = std::vector<bool>(ncells_, false);
        uncovered_ = std::vector<bool>(ncells_, false);
    }

    const mines_belief_t& operator=(const mines_belief_t &bel) {
        *belief_ = *bel.belief_;
        cells_with_mine_ = bel.cells_with_mine_;
        cells_without_mine_ = bel.cells_without_mine_;
        marked_cells_ = bel.marked_cells_;
        flag_ = bel.flag_;
        uncovered_ = bel.uncovered_;
        return *this;
    }

    bool operator==(const mines_belief_t &bel) const {
        return *belief_ == *bel.belief_;
    }
    virtual bool operator!=(const mines_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        belief_->print(os);
    }

    void mark_cell(int cell) {
        if( !marked_cells_[cell] ) {
            if( mine_at(cell) ) {
                marked_cells_[cell] = true;
                cells_with_mine_.push_back(cell);
            }
            if( no_mine_at(cell) ) {
                marked_cells_[cell] = true;
                cells_without_mine_.push_back(cell);
            }
        }
    }

    bool applicable(int action) const {
        int cell = action < ncells_ ? action : action - ncells_;
        return !uncovered_[cell] && !flag_[cell];
    }

    void apply(bool flag, int cell) {
        if( flag ) {
            flag_[cell] = true;
            ++nflags_;
        }
    }
    void apply(int action) {
        apply(action < ncells_, action < ncells_ ? action : action - ncells_);
    }

    void update(bool flag, int cell, int nobs) {
        assert(!flag || (nobs == -1));
        if( !flag ) {
            assert((0 <= nobs) && (nobs < 9));
            mine_filter(cell, nobs, false);
        }
        uncovered_[cell] = true;
        --ncovered_;
    }
    void update(int action, int obs) {
        update(action < ncells_, action < ncells_ ? action : action - ncells_, obs);
    }

    int pop_cell_with_mine() const {
        if( cells_with_mine_.empty() ) {
            return -1;
        } else {
            int cell = cells_with_mine_.back();
            cells_with_mine_.pop_back();
            return cell;
        }
    }

    int pop_cell_without_mine() const {
        if( cells_without_mine_.empty() ) {
            return -1;
        } else {
            int cell = cells_without_mine_.back();
            cells_without_mine_.pop_back();
            return cell;
        }
    }

    // Knowledge-query methods
    bool mine_at(int cell) const {
        return belief_->mine_at(cell);
    }
    bool no_mine_at(int cell) const {
        return belief_->no_mine_at(cell);
    }
    std::pair<int, int> num_surrounding_mines(int cell) const {
        return belief_->num_surrounding_mines(cell);
    }
    float mine_probability(int cell) const {
        float prior = (float)(nmines_ - nflags_) / (float)ncovered_;
        return belief_->mine_probability(cell, prior);
    }

  private:
    void mine_filter(int cell, int nobjs, bool at_least) {
        static std::vector<int> revised_cells;
        belief_->mine_filter(cell, nobjs, at_least, revised_cells);

        // check which revised cell should be marked as
        // containing a mine or not containing a mine
        while( !revised_cells.empty() ) {
            int cell = revised_cells.back();
            revised_cells.pop_back();
            mark_cell(cell);
        }
    }
};

int mines_belief_t::nrows_ = 0;
int mines_belief_t::ncols_ = 0;
int mines_belief_t::ncells_ = 0;
int mines_belief_t::nmines_ = 0;
std::list<mines_belief_t*> mines_belief_t::beliefs_;

inline std::ostream& operator<<(std::ostream &os, const mines_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

