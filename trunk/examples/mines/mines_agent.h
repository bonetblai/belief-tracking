#ifndef AGENT_H
#define AGENT_H

#include "mines_belief.h"

#include <stdlib.h>
#include <cassert>
#include <iostream>

class state_t {
  protected:
    mines_belief_t *belief_;

  public:
    state_t() {
        belief_ = mines_belief_t::allocate();
    }
    explicit state_t(const state_t &state) {
        belief_ = mines_belief_t::allocate();
        *belief_ = *state.belief_;
    }
    state_t(state_t &&state) : belief_(state.belief_) {
        state.belief_ = 0;
    }
    ~state_t() {
        mines_belief_t::deallocate(belief_);
    }

    static void initialize(int nrows, int ncols, int nmines) {
        mines_belief_t::initialize(nrows, ncols, nmines);
    }

    size_t hash() const { return belief_->hash(); }

    int nrows() const { return mines_belief_t::nrows(); }
    int ncols() const { return mines_belief_t::ncols(); }
    int ncells() const { return mines_belief_t::ncells(); }
    int nflags() const { return belief_->nflags(); }
    void set_initial_configuration() { belief_->set_initial_configuration(); }
    bool consistent() const { return belief_->consistent(); }

    int pop_cell_with_mine() const {
        return belief_->pop_cell_with_mine();
    }
    int pop_cell_without_mine() const {
        return belief_->pop_cell_without_mine();
    }
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
        return belief_->mine_probability(cell);
    }

    bool applicable(int action) const {
        return belief_->applicable(action);
    }
    void apply(bool flag, int cell) {
        belief_->apply(flag, cell);
    }
    void apply(int action) {
        belief_->apply(action);
    }
    void update(int flag, int cell, int nobs) {
        belief_->update(flag, cell, nobs);
    }
    void update(int action, int obs) {
        belief_->update(action, obs);
    }
    void apply_action_and_update(int action, int obs) {
        apply(action);
        update(action, obs);
    }

    void print(std::ostream &os) const {
        os << *belief_;
    }

    const state_t& operator=(const state_t &s) {
        *belief_ = *s.belief_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return *belief_ == *s.belief_;
    }
    bool operator<(const state_t &s) const {
        assert(0); return false;
    }
};

struct base_policy_t {

    base_policy_t() { }
    ~base_policy_t() { }

    std::pair<int, bool> operator()(const state_t &state) const {

        // 1st: flag a cell in which we know there is mine
        int cell_with_mine = state.pop_cell_with_mine();
        while( (cell_with_mine != -1) && !state.applicable(cell_with_mine) ) {
            cell_with_mine = state.pop_cell_with_mine();
        }
        if( cell_with_mine != -1 ) {
            assert(state.applicable(cell_with_mine));
            assert(state.mine_at(cell_with_mine));
            //std::cout << "action: flag " << cell_with_mine << std::endl;
            return std::make_pair(cell_with_mine, false);
        }
        //for( int cell = 0; cell < state.ncells(); ++cell ) {
        //    if( state.applicable(cell) )
        //        assert(state.marked_cell(cell) || !state.mine_at(cell));
        //}

        // 2nd: uncover cell in which we known there is no mine
        int cell_without_mine = state.pop_cell_without_mine();
        while( (cell_without_mine != -1) && !state.applicable(cell_without_mine) ) {
            cell_without_mine = state.pop_cell_without_mine();
        }
        if( cell_without_mine != -1 ) {
            assert(state.applicable(cell_without_mine));
            assert(state.no_mine_at(cell_without_mine));
            //std::cout << "action: open " << cell_without_mine << std::endl;
            return std::make_pair(state.ncells() + cell_without_mine, false);
        }
        //for( int cell = 0; cell < state.ncells(); ++cell ) {
        //    if( state.applicable(cell) )
        //        assert(state.marked_cell(cell) || !state.no_mine_at(cell));
        //}

        // 3rd: we must guess a cell to open
        std::vector<int> best;
        best.reserve(state.ncells());

        float best_probability = 1.0;
        for( int cell = 0; cell < state.ncells(); ++cell ) {
            if( state.applicable(cell) ) {
                float probability = state.mine_probability(cell);
                if( probability <= best_probability ) {
                    if( probability < best_probability ) {
                        best_probability = probability;
                        best.clear();
                    }
                    best.push_back(cell);
                }
            }
        }
        if( !best.empty() ) {
            int random_cell = best[lrand48() % best.size()];
            //std::cout << "action: guess " << random_cell << std::endl;
            return std::make_pair(state.ncells() + random_cell, true);
        }

        // 4th: no available action, return -1
        return std::make_pair(-1, true);
    }

};

#endif

