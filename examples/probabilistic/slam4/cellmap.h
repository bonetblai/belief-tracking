/*
 *  Copyright (C) 2015 Universidad Simon Bolivar
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

#ifndef CELLMAP_H
#define CELLMAP_H

#include <cassert>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <stdlib.h>
#include <math.h>

#include <dai/alldai.h>

#include "tracking.h"
#include "action_selection.h"
#include "utils.h"

//#define DEBUG


inline int bit(int n, int k) {
    return (n >> k) & 0x1;
}

inline unsigned popcount(unsigned x) {
    return __builtin_popcountll(x);
}

inline void print_bits(std::ostream &os, int n, int num_bits) {
    for( int i = num_bits - 1; i >= 0; --i )
        os << bit(n, i);
}

inline bool same_column(int i, int j, int ncols) {
    return (i % ncols) == (j % ncols);
}

inline bool same_row(int i, int j, int ncols) {
    return (i / ncols) == (j / ncols);
}

inline int calculate_index_mine_mapping(int slabels, int obs, int loc_type) {
    assert((slabels >= 0) && (slabels < 512));
    assert((obs >= 0) && (obs < 10));
    assert((loc_type >= 0) && (loc_type < 9));
    int index = slabels * 90 + obs * 9 + loc_type;
    assert((index >= 0) && (index < 512 * 10 * 9));
    return index;
}

inline int calculate_index_corridor_slam(int slabels, int obs, int loc_type) {
    assert((slabels >= 0) && (slabels < 8));
    assert((obs >= 0) && (obs < 2));
    assert((loc_type >= 0) && (loc_type < 4));
    int index = slabels * 8 + obs * 4 + loc_type;
    assert((index >= 0) && (index < 8 * 2 * 4));
    return index;
}

struct coord_t {
    static int ncols_;
    int col_;
    int row_;

    coord_t(int col, int row) : col_(col), row_(row) { }
    coord_t(int loc) : col_(loc % ncols_), row_(loc / ncols_) { }
    coord_t(const coord_t &loc) : col_(loc.col_), row_(loc.row_) { }

    bool operator==(const coord_t &loc) const {
        return (col_ == loc.col_) && (row_ == loc.row_);
    }
    bool operator!=(const coord_t &loc) const {
        return !(*this == loc);
    }

    int as_index() const {
        return row_ * ncols_ + col_;
    }

    void print(std::ostream &os) const {
        os << "(" << col_ << "," << row_ << ")";
    }
};

inline std::ostream& operator<<(std::ostream &os, const coord_t &coord) {
    coord.print(os);
    return os;
}

struct cell_t {
    int label_;
    cell_t(int label = 0) : label_(label) { }
};

struct cellmap_t {
    int nrows_;
    int ncols_;
    int nloc_;
    int nvars_;
    int nlabels_;
    int marginals_size_;

    // type of slam problem
    typedef enum { COLOR, MINE_MAPPING_PEAKED, MINE_MAPPING_NON_PEAKED, CORRIDOR } slam_type_t;
    slam_type_t slam_type_;

    float pa_;
    float po_;
    float base_obs_noise_;

    std::vector<int> loc_type_;
    std::vector<std::set<int> > inactive_locs_;
    std::vector<float> probability_obs_mine_mapping_;
    std::vector<float> probability_obs_corridor_slam_;
    std::vector<int> var_offset_;

    std::vector<cell_t> cells_;

    mutable int initial_loc_;

    cellmap_t(int nrows, int ncols, int nlabels, slam_type_t slam_type, float pa, float po, float base_obs_noise = 0.9)
      : nrows_(nrows), ncols_(ncols), nloc_(nrows * ncols),
        nvars_(1 + nloc_), nlabels_(nlabels),
        slam_type_(slam_type),
        pa_(pa), po_(po), base_obs_noise_(base_obs_noise) {
        cells_ = std::vector<cell_t>(nloc_);
        marginals_size_ = 2 * nloc_ + nloc_;
        if( (slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED) ) precompute_stored_information_mine_mapping();
        if( slam_type_ == CORRIDOR ) precompute_stored_information_corridor_slam();
    }
    ~cellmap_t() { }

    int variable_offset(int var) const {
        return 2 * var;
    }

    int variable_size(int var) const {
        return var == nloc_ ? nloc_ : 2;
    }

    int manhattan_distance(const coord_t &loc1, const coord_t &loc2) const {
        return std::abs(loc1.row_ - loc2.row_) + std::abs(loc1.col_ - loc2.col_);
    }
    int manhattan_distance(int loc1, int loc2) const {
        return manhattan_distance(coord_t(loc1), coord_t(loc2));
    }

    void set_labels(const int *labels, size_t size) {
        assert(size == cells_.size());
        for( size_t i = 0; i < size; ++i )
            cells_[i].label_ = labels[i];
    }
    void set_labels(const std::vector<int> &labels) {
        set_labels(&labels[0], labels.size());
    }
    void dump_labels(std::ostream &os) const {
        os << nloc_;
        for( int loc = 0; loc < nloc_; ++loc )
            os << " " << cells_[loc].label_;
    }
    bool read_labels(std::istream &is) {
        int nloc;
        is >> nloc;
        if( nloc == nloc_ ) {
            for( int loc = 0; loc < nloc_; ++loc )
                is >> cells_[loc].label_;
            return true;
        } else {
            std::cout << "MISMATCH: " << nloc_ << " " << nloc << std::endl;
            return false;
        }
    }

    void sample_labels(std::vector<int> &labels) const {
        labels = std::vector<int>(cells_.size(), 0);
        for( size_t i = 0; i < labels.size(); ++i )
            labels[i] = Utils::random(nlabels_);
    }
    void sample_labels() {
        std::vector<int> labels(cells_.size(), 0);
        sample_labels(labels);
        set_labels(labels);
    }

    // action labels
    enum { up, down, right, left, noop };
    static std::string action_label(int action) {
        if( action == 0 )
            return std::string("up");
        else if( action == 1 )
            return std::string("down");
        else if( action == 2 )
            return std::string("right");
        else if( action == 3 )
            return std::string("left");
        else
            return std::string("noop");
    }

    // model for dynamics (locations)
  private:
    float probability_tr_loc_standard(int action, int old_loc, int new_loc, float q) const {
        assert((q >= 0) && (q <= 1));
        if( (action == -1) || (action == noop) ) return old_loc == new_loc ? 1 : 0;

        float p = 0;
        coord_t coord(old_loc), new_coord(new_loc);
        if( action == up ) {
            if( new_coord.col_ == coord.col_ ) {
                if( coord.row_ + 1 < nrows_ ) {
                    p = new_coord.row_ == coord.row_ + 1 ? q : (new_coord.row_ == coord.row_ ? 1 - q : 0);
                } else {
                    assert(coord.row_ + 1 == nrows_);
                    p = new_coord.row_ == coord.row_ ? 1 : 0;
                }
            }
        } else if( action == right ) {
            if( new_coord.row_ == coord.row_ ) {
                if( coord.col_ + 1 < ncols_ ) {
                    p = new_coord.col_ == coord.col_ + 1 ? q : (new_coord.col_ == coord.col_ ? 1 - q : 0);
                } else {
                    assert(coord.col_ + 1 == ncols_);
                    p = new_coord.col_ == coord.col_ ? 1 : 0;
                }
            }
        } else if( action == down ) {
            if( new_coord.col_ == coord.col_ ) {
                if( coord.row_ > 0 ) {
                    p = new_coord.row_ + 1 == coord.row_ ? q : (new_coord.row_ == coord.row_ ? 1 - q : 0);
                } else {
                    assert(coord.row_ == 0);
                    p = new_coord.row_ == coord.row_ ? 1 : 0;
                }
            }
        } else if( action == left ) {
            if( new_coord.row_ == coord.row_ ) {
                if( coord.col_ > 0 ) {
                    p = new_coord.col_ + 1 == coord.col_ ? q : (new_coord.col_ == coord.col_ ? 1 - q : 0);
                } else {
                    assert(coord.col_ == 0);
                    p = new_coord.col_ == coord.col_ ? 1 : 0;
                }
            }
        }
        return p;
    }
    float probability_tr_loc_mine_mapping(int action, int old_loc, int new_loc, float q) const {
        return probability_tr_loc_standard(action, old_loc, new_loc, q);
    }
    float probability_tr_loc_corridor_slam(int action, int old_loc, int new_loc, float q) const {
        return probability_tr_loc_standard(action, old_loc, new_loc, q);
    }

  public:
    float probability_tr_loc(int action, int old_loc, int new_loc, float pa = -1) const {
        float q = pa == -1 ? pa_ : pa;
        if( slam_type_ == COLOR )
            return probability_tr_loc_standard(action, old_loc, new_loc, q);
        else if( (slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED) )
            return probability_tr_loc_mine_mapping(action, old_loc, new_loc, q);
        else
            return probability_tr_loc_corridor_slam(action, old_loc, new_loc, q);
    }

    bool is_noop_action(int action, int loc, float pa = -1) const {
        return probability_tr_loc(action, loc, loc, pa) >= 1 - 1e-5;
    }


  private:
    int sample_loc_standard(int loc, int action, float q) const {
        assert((q >= 0) && (q <= 1));
        if( (action == -1) || (action == noop) ) {
            return loc;
        } else {
            coord_t coord(loc), new_coord(loc);
            if( Utils::uniform() < q ) {
                if( action == up )
                    new_coord = coord_t(coord.col_, coord.row_ + 1 < nrows_ ? coord.row_ + 1 : coord.row_);
                else if( action == right )
                    new_coord = coord_t(coord.col_ + 1 < ncols_ ? coord.col_ + 1 : coord.col_, coord.row_);
                else if( action == down )
                    new_coord = coord_t(coord.col_, coord.row_ > 0 ? coord.row_ - 1 : coord.row_);
                else // action == left
                    new_coord = coord_t(coord.col_ > 0 ? coord.col_ - 1 : coord.col_, coord.row_);
            }
            return new_coord.as_index();
        }
    }
    int sample_loc_mine_mapping(int loc, int action, float q) const {
        return sample_loc_standard(loc, action, q);
    }
    int sample_loc_corridor_slam(int loc, int action, float q) const {
        return sample_loc_standard(loc, action, q);
    }

  public:
    int sample_loc(int loc, int action, float pa = -1) const {
        float q = pa == -1 ? pa_ : pa;
        if( slam_type_ == COLOR )
            return sample_loc_standard(loc, action, q);
        else if( (slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED) )
            return sample_loc_mine_mapping(loc, action, q);
        else
            return sample_loc_corridor_slam(loc, action, q);
    }


    // model for observations
  private:
    // For the standard case, P( obs | label at current loc is 'label' ) is equal to
    // parameter po if obs = label and (1 - po) / (nlabels - 1) otherwise.

    // computes P(obs | label at current loc is 'label')
    float probability_obs_standard(int obs, int /*loc*/, int label, int /*last_action*/) const {
        return label == obs ? po_ : (1 - po_) / float(nlabels_ - 1);
    }
    // computes P(obs | loc, map given by labels)
    float probability_obs_standard(int obs, int loc, const std::vector<int> &labels, int last_action) const {
        return probability_obs_standard(obs, loc, labels[loc], last_action);
    }

    float probability_obs_mine_mapping(int obs, int loc, int slabels, int /*last_action*/) const {
        assert((slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED));
        if( slam_type_ == MINE_MAPPING_NON_PEAKED ) {
            return probability_obs_mine_mapping_[calculate_index_mine_mapping(slabels, obs, loc_type_[loc])];
        } else {
            int label = (slabels >> 4) & 0x1;
            int noise = popcount(slabels);
            float p = po_ + (8 - noise) * base_obs_noise_;
            assert((p >= 0) && (p <= 1));
            return obs == label ? p : 1 - p;
        }
    }
    float probability_obs_mine_mapping(int obs, int loc, const std::vector<int> &labels, int last_action) const {
        assert((slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED));
        if( slam_type_ == MINE_MAPPING_NON_PEAKED ) {
            int slabels = 0;
            int row = loc / ncols_, col = loc % ncols_;
            for( int dr = -1; dr < 2; ++dr ) {
                int nrow = row + dr;
                if( (nrow < 0) || (nrow >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int ncol = col + dc;
                    if( (ncol < 0) || (ncol >= ncols_) ) continue;
                    int bit = (dr + 1) * 3 + (dc + 1);
                    int off_loc = nrow * ncols_ + ncol;
                    int label = labels[off_loc];
                    slabels += (label << bit);
                }
            }
            assert((slabels >= 0) && (slabels < 512));
            return probability_obs_mine_mapping(obs, loc, slabels, last_action);
        } else {
            int noise = 0;
            int row = loc / ncols_, col = loc % ncols_;
            for( int dr = -1; dr < 2; ++dr ) {
                int nrow = row + dr;
                if( (nrow < 0) || (nrow >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    if( (dr == 0) && (dc == 0) ) continue;
                    int ncol = col + dc;
                    if( (ncol < 0) || (ncol >= ncols_) ) continue;
                    int new_loc = nrow * ncols_ + ncol;
                    noise += labels[new_loc] != labels[loc] ? 1 : 0;
                }
            }
            assert(noise < 9);
            float p = po_ + (8 - noise) * base_obs_noise_;
            assert((p >= 0) && (p <= 1));
            return obs == labels[loc] ? p : 1 - p;
        }
    }

    float probability_obs_corridor_slam(int obs, int loc, int slabels, int last_action) const {
        assert(slam_type_ == CORRIDOR);
        int loc_type = (loc == 0 ? 2 : 0) + (loc + 1 == ncols_ ? 1 : 0);
        return probability_obs_corridor_slam_[calculate_index_corridor_slam(slabels, obs, loc_type)];
    }
    float probability_obs_corridor_slam(int obs, int loc, const std::vector<int> &labels, int last_action) const {
        assert(slam_type_ == CORRIDOR);
        assert(loc / ncols_ == 0);
        int slabels = 0;
        if( loc > 0 ) slabels += labels[loc - 1];
        slabels += labels[loc] << 1;
        if( loc + 1 < ncols_ ) slabels += labels[loc + 1] << 2;
        assert((slabels >= 0) && (slabels < 8));
        return probability_obs_corridor_slam(obs, loc, slabels, last_action);
    }

  public:
    float probability_obs(int obs, int loc, int label_or_slabels, int last_action) const {
        if( slam_type_ == COLOR )
            return probability_obs_standard(obs, loc, label_or_slabels, last_action);
        else if( (slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED) )
            return probability_obs_mine_mapping(obs, loc, label_or_slabels, last_action);
        else
            return probability_obs_corridor_slam(obs, loc, label_or_slabels, last_action);
    }
    float probability_obs(int obs, int loc, const std::vector<int> &labels, int last_action) const {
        if( slam_type_ == COLOR )
            return probability_obs_standard(obs, loc, labels, last_action);
        else if( (slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED) )
            return probability_obs_mine_mapping(obs, loc, labels, last_action);
        else
            return probability_obs_corridor_slam(obs, loc, labels, last_action);
    }


  private:
    int sample_label(int label, float q) const {
        assert((q >= 0) && (q <= 1));
        if( Utils::uniform() >= q ) {
            int i = Utils::random(nlabels_ - 1);
            for( int j = 0; j < nlabels_; ++j ) {
                if( (label != j) && (i == 0) ) {
                    label = j;
                    break;
                } else if( label != j ) {
                    --i;
                }
            }
        }
        return label;
    }

    int sample_obs_standard(int loc, int /*last_action*/, float q) const {
        assert(slam_type_ == COLOR);
        assert((loc >= 0) && (loc < nloc_));
        return sample_label(cells_[loc].label_, q);
    }

    int sample_obs_mine_mapping(int loc, int /*last_action*/, float q) const {
        assert((slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED));
        assert((q >= 0) && (q <= 1));
        assert((loc >= 0) && (loc < nloc_));

        int obs = 0;
        int row = loc / ncols_, col = loc % ncols_;

        if( slam_type_ == MINE_MAPPING_NON_PEAKED ) {
            for( int dr = -1; dr < 2; ++dr ) {
                int nrow = row + dr;
                if( (nrow < 0) || (nrow >= nrows_) ) continue; // for outside-of-the-grid cell, sampled value = 0
                for( int dc = -1; dc < 2; ++dc ) {
                    int ncol = col + dc;
                    if( (ncol < 0) || (ncol >= ncols_) ) continue; // for outside-of-the-grid cell, sampled value = 0
                    int new_loc = nrow * ncols_ + ncol;
                    int label = cells_[new_loc].label_;
                    assert((label == 0) || (label == 1));
                    //float p = base_obs_noise_ * (dr != 0 ? q : 1) * (dc != 0 ? q : 1); // this model is too complex
                    float p = base_obs_noise_;
                    obs += Utils::uniform() < p ? label : 1 - label;
                }
            }
            assert((obs >= 0) && (obs < 10));
        } else {
            int loc_label = cells_[loc].label_;
            int noise = 0;
            for( int dr = -1; dr < 2; ++dr ) {
                int nrow = row + dr;
                if( (nrow < 0) || (nrow >= nrows_) ) continue;     // for outside-of-the-grid cell, zero added noise
                for( int dc = -1; dc < 2; ++dc ) {
                    if( (dr == 0) && (dc == 0) ) continue;         // no noise for same loc
                    int ncol = col + dc;
                    if( (ncol < 0) || (ncol >= ncols_) ) continue; // for outside-of-the-grid cell, zero added noise
                    int new_loc = nrow * ncols_ + ncol;
                    int label = cells_[new_loc].label_;
                    noise += label != loc_label ? 1 : 0;
                }
            }
            assert(noise < 9);
            float p = po_ + (8 - noise) * base_obs_noise_;
            assert((p >= 0) && (p <= 1)); //if( (p < 0) || (p > 1) ) std::cout << "error: parameter po must be >= 0.8875, po=" << q << ", p=" << p << std::endl;
            obs = sample_label(loc_label, p);
        }
        return obs;
    }

    int sample_obs_corridor_slam(int loc, int last_action, float q) const {
        assert(slam_type_ == CORRIDOR);
        float p = q;
        if( (loc > 0) && (cells_[loc].label_ != cells_[loc - 1].label_) ) p *= q;
        if( (loc + 1 < ncols_) && (cells_[loc].label_ != cells_[loc + 1].label_) ) p *= q;
        return sample_label(cells_[loc].label_, p);
    }

  public:
    int sample_obs(int loc, int last_action, float po = -1) const {
        float q = po == -1 ? po_ : po;
        if( slam_type_ == COLOR )
            return sample_obs_standard(loc, last_action, q);
        else if( (slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED) )
            return sample_obs_mine_mapping(loc, last_action, q);
        else
            return sample_obs_corridor_slam(loc, last_action, q);
    }

  private:
    // precompute information for efficient computation of observation
    // probabilities.

    // location types
    enum { LOC_CORNER_LO_LE = 0, LOC_CORNER_LO_RI = 1, LOC_CORNER_UP_LE = 2, LOC_CORNER_UP_RI = 3,
           LOC_EDGE_LO = 4, LOC_EDGE_LE = 5, LOC_EDGE_RI = 6, LOC_EDGE_UP = 7,
           LOC_MIDDLE = 8 };

    // wheter a given *relative* loc-type:rel-loc is active or not (belong to grid)
    bool incompatible_loc(int loc, int loc_type) const {
        return inactive_locs_[loc_type].find(loc) != inactive_locs_[loc_type].end();
    }

    // this function is currently not used
    bool incompatible_slabels(int slabels, int loc_type) const {
        for( std::set<int>::const_iterator it = inactive_locs_[loc_type].begin(); it != inactive_locs_[loc_type].end(); ++it ) {
            if( bit(slabels, *it) == 1 )
                return true;
        }
        return false;
    }

    void precompute_stored_information_mine_mapping() {
        assert((slam_type_ == MINE_MAPPING_PEAKED) || (slam_type_ == MINE_MAPPING_NON_PEAKED));

        // location types for each cell: bits for corner and side
        loc_type_ = std::vector<int>(nloc_, 0);
        for( int loc = 0; loc < nloc_; ++loc ) {
            int row = loc / ncols_, col = loc % ncols_;
            if( row == 0 ) {
                if( col == 0 )
                    loc_type_[loc] = LOC_CORNER_LO_LE;
                else if( col == ncols_ )
                    loc_type_[loc] = LOC_CORNER_LO_RI;
                else
                    loc_type_[loc] = LOC_EDGE_LO;
            } else if( row == nrows_ ) {
                if( col == 0 )
                    loc_type_[loc] = LOC_CORNER_UP_LE;
                else if( col == ncols_ )
                    loc_type_[loc] = LOC_CORNER_UP_RI;
                else
                    loc_type_[loc] = LOC_EDGE_UP;
            } else {
                if( col == 0 )
                    loc_type_[loc] = LOC_EDGE_LE;
                else if( col == ncols_ )
                    loc_type_[loc] = LOC_EDGE_RI;
                else
                    loc_type_[loc] = LOC_MIDDLE;
            }
        }

        // inactive locs
        inactive_locs_ = std::vector<std::set<int> >(9);
        inactive_locs_[LOC_CORNER_LO_LE].insert(0);
        inactive_locs_[LOC_CORNER_LO_LE].insert(1);
        inactive_locs_[LOC_CORNER_LO_LE].insert(2);
        inactive_locs_[LOC_CORNER_LO_LE].insert(3);
        inactive_locs_[LOC_CORNER_LO_LE].insert(6);
        inactive_locs_[LOC_CORNER_LO_RI].insert(0);
        inactive_locs_[LOC_CORNER_LO_RI].insert(1);
        inactive_locs_[LOC_CORNER_LO_RI].insert(2);
        inactive_locs_[LOC_CORNER_LO_RI].insert(5);
        inactive_locs_[LOC_CORNER_LO_RI].insert(8);
        inactive_locs_[LOC_CORNER_UP_LE].insert(0);
        inactive_locs_[LOC_CORNER_UP_LE].insert(3);
        inactive_locs_[LOC_CORNER_UP_LE].insert(6);
        inactive_locs_[LOC_CORNER_UP_LE].insert(7);
        inactive_locs_[LOC_CORNER_UP_LE].insert(8);
        inactive_locs_[LOC_CORNER_UP_RI].insert(2);
        inactive_locs_[LOC_CORNER_UP_RI].insert(5);
        inactive_locs_[LOC_CORNER_UP_RI].insert(6);
        inactive_locs_[LOC_CORNER_UP_RI].insert(7);
        inactive_locs_[LOC_CORNER_UP_RI].insert(8);
        inactive_locs_[LOC_EDGE_LO].insert(0);
        inactive_locs_[LOC_EDGE_LO].insert(1);
        inactive_locs_[LOC_EDGE_LO].insert(2);
        inactive_locs_[LOC_EDGE_LE].insert(0);
        inactive_locs_[LOC_EDGE_LE].insert(3);
        inactive_locs_[LOC_EDGE_LE].insert(6);
        inactive_locs_[LOC_EDGE_RI].insert(2);
        inactive_locs_[LOC_EDGE_RI].insert(5);
        inactive_locs_[LOC_EDGE_RI].insert(8);
        inactive_locs_[LOC_EDGE_UP].insert(6);
        inactive_locs_[LOC_EDGE_UP].insert(7);
        inactive_locs_[LOC_EDGE_UP].insert(8);

        // probability for obs
        //
        // We compute P( obs | slabels ) by marginalizing over valuations of
        // the 9 indicator functions for the cells in the beam:
        //
        //   P( obs | slabels ) = \sum_{val} P( obs, val | slabels)
        //                      = \sum_{val} P( obs | val, slabels) P( val | slabels)
        //                      = \sum_{val} P( obs | val ) \prod_{loc} P( val[loc] | slabels[loc] )
        //                      = \sum_{val} [[ obs = #{ loc : val[loc] = 1 } ]] \prod_{loc} P( val[loc] | slabels[loc] )

        if( slam_type_ == MINE_MAPPING_NON_PEAKED ) {
            // dimension is product of 512 valuations, 10 obs and 9 loc-types
            probability_obs_mine_mapping_ = std::vector<float>(512 * 10 * 9, 0);
            for( int loc_type = 0; loc_type < 9; ++loc_type ) {
                for( int obs = 0; obs < 10; ++obs ) {
                    for( int slabels = 0; slabels < 512; ++slabels ) {
                        //if( incompatible_slabels(slabels, loc_type) ) continue;
                        int index = calculate_index_mine_mapping(slabels, obs, loc_type);
                        for( int valuation = 0; valuation < 512; ++valuation ) {
                            //if( incompatible_slabels(valuation, loc_type) ) continue;
                            if( int(popcount(valuation)) != obs ) continue;
                            float p = 1;
                            for( int rloc = 0; rloc < 9; ++rloc ) { // computes \prod_{loc} P( val[loc] | slabels[loc] ) onto p
                                if( incompatible_loc(rloc, loc_type) ) {
                                    //continue;
                                    p *= bit(valuation, rloc) == 0 ? 0.99 : 0.01;
                                } else {
                                    // loc=4 refers to the center of the 3x3 window
                                    //float q = base_obs_noise_ * (same_column(rloc, 4, 3) ? 1 : po_) * (same_row(rloc, 4, 3) ? 1 : po_); // this model is too complex
                                    float q = base_obs_noise_;
                                    p *= bit(valuation, rloc) == bit(slabels, rloc) ? q : 1 - q;
                                }
                            }
                            probability_obs_mine_mapping_[index] += p;
                        }
                        assert(probability_obs_mine_mapping_[index] <= 1);
                        if( probability_obs_mine_mapping_[index] == 0 ) {
                            std::cout << "error: P(obs=" << obs
                                      << "|slabels=" << slabels
                                      << ", loc-type=" << loc_type
                                      << ") = " << probability_obs_mine_mapping_[index]
                                      << std::endl;
                            assert(probability_obs_mine_mapping_[index] > 0);
                        } else {
#if 0
                            std::cout << "# P(obs=" << obs
                                      << "|slabels=" << slabels
                                      << ", loc_type=" << loc_type
                                      << ")=" << probability_obs_mine_mapping_[index]
                                      << std::endl;
#endif
                        }
                    }
                }
            }

            for( int obs = 0; obs < 10; ++obs ) {
                float p = probability_obs_mine_mapping_[calculate_index_mine_mapping(0, obs, LOC_MIDDLE)];
                std::cout << "# P(obs=" << obs << "|slabels=0,MIDDLE)=" << p << std::endl;
            }
            for( int obs = 0; obs < 10; ++obs ) {
                float p = probability_obs_mine_mapping_[calculate_index_mine_mapping(0, obs, LOC_CORNER_UP_RI)];
                std::cout << "# P(obs=" << obs << "|slabels=0,CORNER)=" << p << std::endl;
            }
            for( int obs = 0; obs < 10; ++obs ) {
                float p = probability_obs_mine_mapping_[calculate_index_mine_mapping(0, obs, LOC_EDGE_UP)];
                std::cout << "# P(obs=" << obs << "|slabels=0,EDGE)=" << p << std::endl;
            }

            // check probabilities for observation: \sum_{obs} P( obs | slabels ) = 1 for all slabels
            for( int loc_type = 0; loc_type < 9; ++loc_type ) {
                for( int slabels = 0; slabels < 512; ++slabels ) {
                    //if( incompatible_slabels(slabels, loc_type) ) continue;
                    float sum = 0;
                    for( int obs = 0; obs < 10; ++obs )
                        sum += probability_obs_mine_mapping_[calculate_index_mine_mapping(slabels, obs, loc_type)];
                    if( fabs(sum - 1) >= 1e-6 ) std::cout << "warning: |sum - 1| >= 1e-6,  sum=" << sum << std::endl;
                    //assert(fabs(sum - 1) < 1e-5);
#ifdef DEBUG
                    std::cout << "total-mass[lt=" << loc_type << ",labels=" << slabels << "] = " << sum << std::endl;
#endif
                }
            }

#if 0
            // check that the most plausible obs correspond to number of ore bits
            for( int loc_type = 0; loc_type < 9; ++loc_type ) {
                for( int slabels = 0; slabels < 512; ++slabels ) {
                    int best_obs = 0;
                    float best_obs_prob = 0;
                    for( int obs = 0; obs < 10; ++obs ) {
                        float p = probability_obs_mine_mapping_[calculate_index_mine_mapping(slabels, obs, loc_type)];
                        if( p > best_obs_prob ) {
                            best_obs = obs;
                            best_obs_prob = p;
                        }
                    }
                    int correct_obs = popcount(slabels);
                    if( best_obs != correct_obs ) {
                        std::cout << "warning: most plausible obs for (loc-type=" << loc_type
                                  << ", slabels=" << slabels
                                  << ", bits=|";
                        print_bits(std::cout, slabels, 9);
                        std::cout << "|) is " << best_obs
                                  << " with p=" << best_obs_prob
                                  << "; correct obs " << correct_obs
                                  << " has p="
                                  << probability_obs_mine_mapping_[calculate_index_mine_mapping(slabels, correct_obs, loc_type)]
                                  << std::endl;
                    }
                }
            }
#endif
        } else {
            base_obs_noise_ = (1.0 - po_) / 9.0;
        }

        // calculate var offsets
        var_offset_ = std::vector<int>(nloc_ * nloc_, -1);
        for( int loc = 0; loc < nloc_; ++loc ) {
            int row = loc / ncols_, col = loc % ncols_;
            for( int dr = -1; dr < 2; ++dr ) {
                int nrow = row + dr;
                if( (nrow < 0) || (nrow >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int ncol = col + dc;
                    if( (ncol < 0) || (ncol >= ncols_) ) continue;
                    int new_loc = nrow * ncols_ + ncol;
                    int offset = (dr + 1) * 3 + (dc + 1);
                    var_offset_[loc * nloc_ + new_loc] = offset;
                }
            }
        }
    }

    void precompute_stored_information_corridor_slam() {
        // dimension is product of 8 valuations, 2 obs and 4 loc-types
        probability_obs_corridor_slam_ = std::vector<float>(8 * 2 * 4, 0);
        for( int loc_type = 0; loc_type < 4; ++loc_type ) {
            for( int obs = 0; obs < 2; ++obs ) {
                for( int slabels = 0; slabels < 8; ++slabels ) {
                    int label = (slabels >> 1) & 1;
                    int index = calculate_index_corridor_slam(slabels, obs, loc_type);
                    float p = po_;
                    if( ((loc_type & 1) == 0) && ((slabels & 1) != label) ) p *= po_;
                    if( ((loc_type & 2) == 0) && (((slabels >> 2) & 1) != label) ) p *= po_;
                    probability_obs_corridor_slam_[index] = obs == label ? p : 1 - p;

                    std::cout << "# P( obs=" << obs << " | loc=" << loc_type << ":["
                              << ((loc_type & 2) >> 1) << (loc_type & 1) << "], slabels=" << slabels << ":[";
                    if( (loc_type & 2) == 0 )
                        std::cout << (slabels & 1);
                    else
                        std::cout << "-";
                    std::cout << ((slabels >> 1) & 1);
                    if( (loc_type & 1) == 0 )
                        std::cout << ((slabels >> 2) & 1);
                    else
                        std::cout << "-";
                    std::cout << "] ) = " << probability_obs_corridor_slam_[index] << std::endl;
                }
            }
        }
    }

  public:
    int var_offset(int loc, int var_id) const {
        assert((loc >= 0) && (loc < nloc_));
        assert((var_id >= 0) && (var_id < nloc_));
        return var_offset_[loc * nloc_ + var_id];
    }

    // tracking function
    void run_initialize(std::vector<tracking_t<cellmap_t>*> &trackers, std::vector<repository_t> &repos) const {
        repos.clear();
        repos.reserve(trackers.size());
        for( size_t i = 0; i < trackers.size(); ++i ) {
            tracking_t<cellmap_t> &tracker = *trackers[i];
            tracker.add_run();
            tracker.initialize();
            repos.push_back(repository_t());
        }
    }
    void advance_step(std::vector<repository_t> &repos, int last_action, int obs, std::vector<tracking_t<cellmap_t>*> &trackers, bool print_marginals = false) const {
        assert(repos.size() == trackers.size());
        for( size_t i = 0; i < trackers.size(); ++i ) {
            float start_time = Utils::read_time_in_seconds();
            tracking_t<cellmap_t> &tracker = *trackers[i];
            tracker.update(last_action, obs);
            tracker.calculate_marginals();
            tracker.store_marginals(repos[i]);
            float elapsed_time = Utils::read_time_in_seconds() - start_time;
            if( print_marginals ) tracker.print_marginals(std::cout, repos[i]);
            tracker.add_elapsed_time(elapsed_time);
        }
    }

    void print_labels(std::ostream &os) const {
        os << "[";
        for( int i = 0; i < nrows_ * ncols_; ++i ) {
            os << " " << cells_[i].label_;
            if( (i + 1 < nrows_ * ncols_) && (((i + 1) % ncols_) == 0) )
                os << " |";
        }
        os << "]";
    }

    struct execution_step_t {
        int loc_;
        int obs_;
        int last_action_;
        execution_step_t(int loc = 0, int obs = 0, int last_action = 0)
          : loc_(loc), obs_(obs), last_action_(last_action) { }
        void print(std::ostream &os) const {
            std::string action_label = cellmap_t::action_label(last_action_);
            os << "[" << coord_t(loc_) << "," << obs_ << "," << action_label << "]";
        }
        void dump(std::ostream &os) const {
            os << loc_ << " " << obs_ << " " << last_action_;
        }
        bool read(std::istream &is) {
            is >> loc_ >> obs_ >> last_action_;
            return true;
        }
    };
    struct execution_t : public std::vector<execution_step_t> {
        execution_t() { }
        execution_t(int n, const execution_step_t &step)
          : std::vector<execution_step_t>(n, step) { }
        void print(std::ostream &os) const {
            os << "<";
            for( int i = 0; i < int(size()); ++i ) {
                (*this)[i].print(os);
                if( i < int(size()) - 1 ) os << ",";
            }
            os << ">";
        }
        void dump(std::ostream &os) const {
            os << int(size());
            for( int i = 0; i < int(size()); ++i ) {
                os << " ";
                (*this)[i].dump(os);
            }
        }
        bool read(std::istream &is) {
            int nsteps = 0;
            is >> nsteps;
            clear();
            reserve(nsteps);
            for( int i = 0; i < nsteps; ++i ) {
                execution_step_t step;
                if( !step.read(is) ) break;
                push_back(step);
            }
            return nsteps == int(size());
        }
    };

    int random_action() const {
        int index = Utils::random();
        if( (ncols_ == 1) || (nrows_ == 1) ) {
            index = index % 2;
            return ncols_ == 1 ? index : 2 + index;
        } else {
            return index % 4;
        }
    }

    int action_for(const coord_t &current, const coord_t &target) const {
        if( current == target ) {
            return noop;
        } else if( target.row_ != current.row_ ) {
            if( target.col_ != current.col_ ) {
                // row and col are different, first try to get to the right column
                return target.col_ > current.col_ ? right : left;
            } else {
                // action must be either up or down
                return target.row_ > current.row_ ? up : down;
            }
        } else {
            assert(target.col_ != current.col_);
            return target.col_ > current.col_ ? right : left;
        }
    }
    int action_for(int current, int target) const {
        return action_for(coord_t(current), coord_t(target));
    }

    void compute_random_execution(int initial_loc, int length, execution_t &execution) const {
        execution.reserve(length + 1);
        execution.push_back(execution_step_t(initial_loc, sample_obs(initial_loc, -1), -1));
        for( int step = 0; step < length; ++step ) {
            int current_loc = execution.back().loc_;
            int action = random_action();
            int new_loc = sample_loc(current_loc, action);
            int obs = sample_obs(new_loc, action);
            execution.push_back(execution_step_t(new_loc, obs, action));
        }
    }

    void compute_covering_execution(int initial_loc, execution_t &execution, int loops = 1) const {
        execution.reserve(loops * nloc_ + 1);
        execution.push_back(execution_step_t(initial_loc, sample_obs(initial_loc, -1), -1));

        while( loops > 0 ) {
            std::set<int> cells_to_visit;
            for( int loc = 0; loc < nloc_; ++loc )
                cells_to_visit.insert(loc);

            cells_to_visit.erase(execution.back().loc_);
            while( !cells_to_visit.empty() ) {
                int target = *cells_to_visit.begin();
                int current_loc = execution.back().loc_;
                int action = action_for(current_loc, target);
                int new_loc = sample_loc(current_loc, action);
                int obs = sample_obs(new_loc, action);
                execution.push_back(execution_step_t(new_loc, obs, action));
                cells_to_visit.erase(new_loc);
            }
            --loops;
        }
    }

    void run_execution(std::vector<repository_t> &repos,
                       const execution_t &input_execution,
                       execution_t &output_execution,
                       int nsteps,
                       const action_selection_t<cellmap_t> *policy,
                       std::vector<tracking_t<cellmap_t>*> &trackers,
                       bool print_marginals = false) {
        // input execution must contain at least one initial step
        assert(!input_execution.empty());
        const execution_step_t &initial_step = input_execution[0];
        int hidden_loc = initial_step.loc_;
        int obs = initial_step.obs_ == -1 ? cells_[hidden_loc].label_ : initial_step.obs_;
        assert(initial_step.last_action_ == -1);

        // initialize tracking algorithms and output execution
        std::cout << "# initializing... " << std::flush;
        output_execution.clear();
        run_initialize(trackers, repos);
        output_execution.push_back(initial_step);
        advance_step(repos, -1, obs, trackers, print_marginals);
        std::cout << std::endl;

        // run execution
        std::cout << "# steps:" << std::flush;
        for( size_t t = 1; true; ++t ) {
            std::cout << /*std::endl <<*/ " " << t << std::flush;
            if( (policy == 0) && (t >= input_execution.size()) ) break;
            if( (policy != 0) && (t >= size_t(nsteps)) ) break;

            int last_action = -1;
            int obs = -1;

            // select next action, update hidden loc, and sample observation
            if( policy == 0 ) {
                const execution_step_t &step = input_execution[t];
                hidden_loc = step.loc_;
                obs = step.obs_;
                last_action = step.last_action_;
            } else {
                assert(!trackers.empty());
                last_action = policy->select_action(trackers[0]);
                if( last_action == -1 ) break;
                hidden_loc = sample_loc(hidden_loc, last_action);
                obs = sample_obs(hidden_loc, last_action);
                //std::cout << "[hidden-loc=" << coord_t(hidden_loc) << ",obs=" << obs << "]";
            }

            // update tracking
            output_execution.push_back(execution_step_t(hidden_loc, obs, last_action));
            advance_step(repos, last_action, obs, trackers, print_marginals);
        }
        std::cout << std::endl;
    }
    void run_execution(std::vector<repository_t> &repos,
                       const execution_t &input_execution,
                       execution_t &output_execution,
                       std::vector<tracking_t<cellmap_t>*> &trackers,
                       bool print_marginals = false) {
        run_execution(repos, input_execution, output_execution, 0, 0, trackers, print_marginals);
    }
    void run_execution(std::vector<repository_t> &repos,
                       execution_t &output_execution,
                       int initial_loc,
                       int nsteps,
                       const action_selection_t<cellmap_t> &policy,
                       std::vector<tracking_t<cellmap_t>*> &trackers,
                       bool print_marginals = false) {
        execution_step_t initial_step(initial_loc, -1, -1);
        execution_t empty_execution(1, initial_step);
        run_execution(repos, empty_execution, output_execution, nsteps, &policy, trackers, print_marginals);
    }

    // scoring of tracking algorithms
    typedef int score_t;

    score_t compute_score(int current_loc, const std::set<int> &relevant_cells, const tracking_t<cellmap_t> &tracker) const {
        score_t score = 0;
        for( int var = 0; var < 1 + nrows_ * ncols_; ++var ) {
            if( relevant_cells.find(var) != relevant_cells.end() ) {
                dai::Factor marginal;
                tracker.get_marginal(var, marginal);
                float max_probability = 0;
                std::vector<int> most_probable;
                for( size_t i = 0; i < marginal.nrStates(); ++i ) {
                    if( marginal[i] >= max_probability ) {
                        if( marginal[i] > max_probability )
                            most_probable.clear();
                        most_probable.push_back(i);
                        max_probability = marginal[i];
                    }
                }
                if( most_probable.size() == 1 ) {
                    if( var < nrows_ * ncols_ ) {
                        score += cells_[var].label_ == most_probable[0] ? 1 : 0;
                    } else {
                        score += current_loc == most_probable[0] ? 1 : 0;
                    }
                }
            }
        }
        return score;
    }
    score_t compute_score(const execution_t &execution, const tracking_t<cellmap_t> &tracker) const {
        std::set<int> relevant_cells;
        for( size_t i = 0; i < execution.size(); ++i )
            relevant_cells.insert(execution[i].loc_);
        int current_loc = execution.back().loc_;
        return compute_score(current_loc, relevant_cells, tracker);
    }

    void compute_scores(const execution_t &execution, const std::vector<tracking_t<cellmap_t>*> &trackers, std::vector<score_t> &scores) const {
        scores.clear();
        for( size_t i = 0; i < trackers.size(); ++i )
            scores.push_back(compute_score(execution, *trackers[i]));
    }

    // R plots
    void generate_R_plot(std::ostream &os, const tracking_t<cellmap_t> &tracker, const repository_t &repository) const {
        for( size_t t = 0; t < repository.size(); ++t ) {
            const float *loc_marginal = tracker.stored_marginal(repository, t, nloc_);
            os << "mar_" << tracker.name_ << "_t" << t << " <- c(";
            for( int value = 0; value < nloc_; ++value ) {
                os << loc_marginal[value];
                if( 1 + value < nloc_ ) os << ", ";
            }
            os << ");" << std::endl;
        }
        os << "mar_" << tracker.name_ << " <- c(";
        for( size_t t = 0; t < repository.size(); ++t )
            os << "mar_" << tracker.name_ << "_t" << t << (t + 1 < repository.size() ? ", " : "");
        os << ");" << std::endl;

        int ncols_in_plot = nrows_ * ncols_;
        os << "dmf <- melt(as.data.frame(t(matrix(mar_" << tracker.name_ << ", ncol=" << ncols_in_plot << ", byrow=T))));" << std::endl
           << "dmf$y <- rep(1:" << ncols_in_plot << ", " << repository.size() << ");" << std::endl
           << "plot_" << tracker.name_ << " <- ggplot(dmf,aes(y=variable, x=y)) + "
           << "geom_tile(aes(fill=value)) + "
           //<< "geom_text(aes(label=ifelse(value>.01, trunc(100*value, digits=2)/100, \"\")), size=3, angle=0, color=\"white\") +"
           << "labs(y=\"time step\", x=\"location\") + "
           << "theme_minimal() + "
           //<< "scale_x_discrete(limits=1:" << ncols_in_plot << ") + "
           //<< "scale_y_discrete(labels=paste(\"t=\", 0:" << tracker.marginals_.size() << ", sep=\"\"));"
           //<< "scale_y_discrete(labels=rep(\"\", " << tracker.marginals_.size() << "));"
           << std::endl;

        // save in plot in pdf
        os << "pdf(\"marginals_" << tracker.name_ << ".pdf\");" << std::endl
           << "show(plot_" << tracker.name_ << ")" << std::endl
           << "dev.off();"
           << std::endl;
    }
};

inline std::ostream& operator<<(std::ostream &os, const cellmap_t::execution_t &execution) {
    execution.print(os);
    return os;
}

#undef DEBUG

#endif

