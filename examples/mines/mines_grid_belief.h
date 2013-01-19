#ifndef MINES_GRID_BELIEF_H
#define MINES_GRID_BELIEF_H

#include <cell_beam.h>
#include <arc_consistency.h>

#include <cassert>
#include <iostream>
#include <vector>

class grid_arc_consistency_t : public CSP::arc_consistency_t<cell_beam_t> {
    mutable int row_x_, row_y_, row_diff_;
    mutable int col_x_, col_y_, col_diff_;
    static std::vector<int> cells_to_revise_[25];

    // disallow copy constructor
    explicit grid_arc_consistency_t(const grid_arc_consistency_t &);
    explicit grid_arc_consistency_t(grid_arc_consistency_t &&);

  public:
    grid_arc_consistency_t();
    virtual ~grid_arc_consistency_t() { }
    static void initialize(int neighbourhood);

    virtual void arc_reduce_preprocessing_0(int var_x, int var_y);
    virtual void arc_reduce_preprocessing_1(int var_x, int val_x) { }
    virtual void arc_reduce_postprocessing(int var_x, int var_y) { }

    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const {
        int index = (row_diff_ + 2) * 5 + (col_diff_ + 2);
        assert((0 <= index) && (index < 25));
        for( int k = 0, ksz = cells_to_revise_[index].size(); k < ksz; ++k ) {
            int entry = cells_to_revise_[index][k];
            int i = entry / 9, j = entry % 9;
            if( ((val_x >> i) & 0x1) != ((val_y >> j) & 0x1) ) {
                return false;
            }
        }
        return true;
    }

    const grid_arc_consistency_t& operator=(const grid_arc_consistency_t &arc) {
        assert(0); // shouldn't be called
        return arc;
    }
    bool operator==(const grid_arc_consistency_t &arc) {
        assert(0); // shouldn't be called
        return false;
    }
};

class grid_belief_t {
  protected:
    static int nrows_;
    static int ncols_;
    static int ncells_;
    static int *types_;
    static CSP::constraint_digraph_t cg_;

  protected:
    cell_beam_t **beams_;
    grid_arc_consistency_t mines_;

  public:
    grid_belief_t()
      : beams_(new cell_beam_t*[ncells_]) {
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                beams_[cell] = new cell_beam_t(r, c, grid_belief_t::types_[cell]);
                mines_.set_domain(cell, beams_[cell]);
            }
        }
    }
    grid_belief_t(const grid_belief_t &bel)
      : beams_(new cell_beam_t*[ncells_]) {
        for( int cell = 0; cell < ncells_; ++cell ) {
            beams_[cell] = new cell_beam_t(*bel.beams_[cell]);
            mines_.set_domain(cell, beams_[cell]);
        }
    }
    grid_belief_t(grid_belief_t &&bel)
      : beams_(bel.beams_) {
        bel.beams_ = 0;
        for( int cell = 0; cell < ncells_; ++cell ) {
            mines_.set_domain(cell, beams_[cell]);
            bel.mines_.set_domain(cell, 0);
        }
    }
    virtual ~grid_belief_t() {
        for( int cell = 0; cell < ncells_; ++cell ) {
            delete beams_[cell];
        }
        delete[] beams_;
    }

    enum { manhattan_neighbourhood = 186, octile_neighbourhood = 511 };

    // neighbourhood is used to set up the cells to revise when
    // doing arc consistency. Two values had been validated (tested):
    // the value 511 for octile neighbourboods, and the value
    // 186 for 'manhattan' neighbourhoods.
    static void initialize(int nrows, int ncols, int neighbourhood) {
        static bool initialized = false;
        if( initialized && (nrows_ == nrows) && (ncols_ == ncols) ) return;
        initialized = true;

        std::cout << "grid_belief_t: initialization: "
                  << "nrows=" << nrows
                  << ", ncols=" << ncols
                  << ", neighbourhood=" << neighbourhood
                  << std::endl;

        nrows_ = nrows;
        ncols_ = ncols;
        ncells_ = nrows_ * ncols_;

        // beam types
        types_ = new int[ncells_];
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int type = 0;
                if( r == 0 ) type += cell_beam_t::BOTTOM;
                if( r == nrows_ - 1 ) type += cell_beam_t::TOP;
                if( c == 0 ) type += cell_beam_t::LEFT;
                if( c == ncols_ - 1 ) type += cell_beam_t::RIGHT;
                types_[r * ncols_ + c] = type;
            }
        }

        // constraint graph
        construct_constraint_graph(nrows_, ncols_, neighbourhood);
        grid_arc_consistency_t::initialize(neighbourhood);
    }

    static bool skip_cell(int cell, int i) {
        int type = types_[cell];
        if( (type & cell_beam_t::TOP) && ((i == 6) || (i == 7) || (i == 8)) ) return true;
        if( (type & cell_beam_t::BOTTOM) && ((i == 0) || (i == 1) || (i == 2)) ) return true;
        if( (type & cell_beam_t::LEFT) && ((i == 0) || (i == 3) || (i == 6)) ) return true;
        if( (type & cell_beam_t::RIGHT) && ((i == 2) || (i == 5) || (i == 8)) ) return true;
        return false;
    }

    static int nrows() { return nrows_; }
    static int ncols() { return ncols_; }
    static const CSP::constraint_digraph_t& cg() { return cg_; }

    bool consistent() const {
        for( int cell = 0; cell < ncells_; ++cell ) {
            if( beams_[cell]->empty() ) return false;
        }
        return true;
    }

    void clear() {
        for( int cell = 0; cell < ncells_; ++cell ) {
            beams_[cell]->clear();
        }
    }

    void set_initial_configuration() {
        for( int cell = 0; cell < ncells_; ++cell ) {
            beams_[cell]->set_initial_configuration(grid_belief_t::octile_neighbourhood);
        }
    }

    void filter(std::vector<cell_beam_t*> &beams, int cell, int nobjs, bool at_least = false) {
        assert((0 <= cell) && (cell < ncells_));
        beams[cell]->filter(nobjs, at_least);
    }

    const grid_belief_t& operator=(const grid_belief_t &bel) {
        for( int cell = 0; cell < ncells_; ++cell ) {
            assert(beams_[cell] != 0);
            *beams_[cell] = *bel.beams_[cell];
        }
        return *this;
    }

    bool operator==(const grid_belief_t &bel) const {
        for( int cell = 0; cell < ncells_; ++cell ) {
            if( *beams_[cell] != *bel.beams_[cell] ) return false;
        }
        return true;
    }
    virtual bool operator!=(const grid_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                os << "cell(" << c << "," << r << ")=" << *beams_[cell] << std::endl;
            }
        }
    }

    bool mine_at(int cell) const {
        return beams_[cell]->obj_at();
    }

    bool no_mine_at(int cell) const {
        return beams_[cell]->no_obj_at();
    }

    std::pair<int, int> num_surrounding_mines(int cell) const {
        return beams_[cell]->num_surrounding_objs();
    }

    float mine_probability(int cell, float prior) const {
        int row = cell / ncols_, col = cell % ncols_;
        float probability = 0.0, n = 0;
        for( int drow = -1; drow < 2; ++drow ) {
            int r = row + drow;
            if( (r < 0) || (r >= nrows_) ) continue;
            for( int dcol = -1; dcol < 2; ++dcol ) {
                int c = col + dcol;
                if( (c < 0) || (c >= ncols_) ) continue;
                int beam = r * ncols_ + c;
                // translate (dc,dr) to coordinate system centered at beam
                int bit_index = (1 - drow) * 3 + (1 - dcol);
                float p = beams_[beam]->obj_probability(prior, bit_index);
                probability += p;
                ++n;
            }
        }
        return probability / n;
    }

    void mine_filter(int cell, int nobjs, bool at_least, std::vector<int> &revised_cells) {
        revised_cells.clear();
        beams_[cell]->filter(nobjs, at_least);
        mines_.add_to_worklist(cell);
        mines_.ac3(revised_cells);
    }

    // Consistency methods
    static void construct_constraint_graph(int nrows, int ncols, int neighbourhood) {
        cg_.create_empty_graph(nrows * ncols);
        for( int r = 0; r < nrows; ++r ) {
            for( int c = 0; c < ncols; ++c ) {
                int cell = r * ncols + c;
                cg_.reserve_edge_list(cell, 25);
                for( int dr = -2; dr < 3; ++dr ) {
                    if( (r + dr < 0) || (r + dr >= nrows) ) continue;
                    for( int dc = -2; dc < 3; ++dc ) {
                        if( (c + dc < 0) || (c + dc >= ncols) ) continue;
                        if( (neighbourhood == manhattan_neighbourhood) &&
                            ((dr == -2) || (dr == 2)) && (dc != 0) )
                            continue;
                        if( (neighbourhood == manhattan_neighbourhood) &&
                            ((dc == -2) || (dc == 2)) && (dr != 0) )
                            continue;
                        int ncell = (r + dr) * ncols + (c + dc);
                        if( ncell != cell ) {
                            cg_.add_edge(ncell, cell);
                        }
                    }
                }
            }
        }
        std::cout << "cg: #edges=" << cg_.nedges() << std::endl;
    }
};

int grid_belief_t::nrows_ = 0;
int grid_belief_t::ncols_ = 0;
int grid_belief_t::ncells_ = 0;
int *grid_belief_t::types_ = 0;
CSP::constraint_digraph_t grid_belief_t::cg_;

grid_arc_consistency_t::grid_arc_consistency_t()
  : CSP::arc_consistency_t<cell_beam_t>(grid_belief_t::cg()) {
}

void grid_arc_consistency_t::initialize(int neighbourhood) {
    for( int k = 0; k < 25; ++k ) {
        cells_to_revise_[k].reserve(9);
        int row_diff = (k / 5) - 2, col_diff = (k % 5) - 2;
        int beam_x = 2 * 5 + 2;
        int beam_y = (2 + row_diff) * 5 + (2 + col_diff);
        for( int i = 0; i < 9; ++i ) {
            if( (neighbourhood & (1 << i)) == 0 ) continue;
            int cell_x = beam_x + (((i / 3) - 1) * 5 + ((i % 3) - 1));
            for( int j = 0; j < 9; ++j ) {
                if( (neighbourhood & (1 << j)) == 0 ) continue;
                int cell_y = beam_y + (((j / 3) - 1) * 5 + ((j % 3) - 1));
                if( cell_x == cell_y ) {
                    int entry = i * 9 + j;
                    cells_to_revise_[k].push_back(entry);
                }
            }
        }
    }
}

void grid_arc_consistency_t::arc_reduce_preprocessing_0(int var_x, int var_y) {
    row_x_ = var_x / grid_belief_t::ncols(), col_x_ = var_x % grid_belief_t::ncols();
    row_y_ = var_y / grid_belief_t::ncols(), col_y_ = var_y % grid_belief_t::ncols();
    row_diff_ = row_y_ - row_x_;
    col_diff_ = col_y_ - col_x_;
}

std::vector<int> grid_arc_consistency_t::cells_to_revise_[25];

#endif

