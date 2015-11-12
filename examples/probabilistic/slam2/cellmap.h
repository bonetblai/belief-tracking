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

//#define DEBUG


inline int bit(int n, int k) {
    return (n >> k) & 0x1;
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

inline int calculate_index(int slabels, int obs, int loc_type) {
    assert((slabels >= 0) && (slabels < 512));
    assert((obs >= 0) && (obs < 10));
    assert((loc_type >= 0) && (loc_type < 9));
    int index = slabels * 90 + obs * 9 + loc_type;
    assert((index >= 0) && (index < 512 * 90));
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

    // tells whether this problem is colortile-slam or ore-slam
    bool oreslam_;

    float pa_;
    float po_;
    float base_obs_noise_;

    std::vector<int> num_bits_;
    std::vector<int> loc_type_;
    std::vector<std::set<int> > inactive_locs_;
    std::vector<float> probability_obs_oreslam_;
    std::vector<int> var_offset_;

    std::vector<cell_t> cells_;

    mutable int initial_loc_;

    cellmap_t(int nrows, int ncols, int nlabels, bool oreslam, float pa, float po, float base_obs_noise = 0.9)
      : nrows_(nrows), ncols_(ncols), nloc_(nrows * ncols),
        nvars_(1 + nloc_), nlabels_(nlabels),
        oreslam_(oreslam),
        pa_(pa), po_(po), base_obs_noise_(base_obs_noise) {
        cells_ = std::vector<cell_t>(nloc_);
        marginals_size_ = 2 * nloc_ + nloc_;
        if( oreslam_ )
            precompute_stored_information_oreslam(); // precompute stored information
    }
    ~cellmap_t() { }

    int variable_offset(int var) const {
        return 2 * var;
    }

    int variable_size(int var) const {
        return var == nloc_ ? nloc_ : 2;
    }

    void set_labels(const int *labels, size_t size) {
        assert(size == cells_.size());
        for( size_t i = 0; i < size; ++i )
            cells_[i].label_ = labels[i];
    }
    void set_labels(const std::vector<int> &labels) {
        set_labels(&labels[0], labels.size());
    }

    void sample_labels(std::vector<int> &labels) const {
        labels = std::vector<int>(cells_.size(), 0);
        for( size_t i = 0; i < labels.size(); ++i )
            labels[i] = lrand48() % nlabels_;
    }
    void sample_labels() {
        std::vector<int> labels(cells_.size(), 0);
        sample_labels(labels);
        set_labels(labels);
    }

    // action labels
    enum { up, down, right, left, noop };
    std::string action_label(int action) const {
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
        if( action == noop ) return old_loc == new_loc ? 1 : 0;

        float p = 0;
        coord_t coord(old_loc), new_coord(new_loc);
        if( action == -1 ) {
            p = old_loc == new_loc ? 1 : 0;
        } else if( action == up ) {
            if( new_coord.col_ != coord.col_ ) {
                p = 0;
            } else {
                if( coord.row_ + 1 < nrows_ ) {
                    p = new_coord.row_ == coord.row_ + 1 ? q : (new_coord.row_ == coord.row_ ? 1 - q : 0);
                } else {
                    assert(coord.row_ + 1 == nrows_);
                    p = new_coord.row_ == coord.row_ ? 1 : 0;
                }
            }
        } else if( action == right ) {
            if( new_coord.row_ != coord.row_ ) {
                p = 0;
            } else {
                if( coord.col_ + 1 < ncols_ ) {
                    p = new_coord.col_ == coord.col_ + 1 ? q : (new_coord.col_ == coord.col_ ? 1 - q : 0);
                } else {
                    assert(coord.col_ + 1 == ncols_);
                    p = new_coord.col_ == coord.col_ ? 1 : 0;
                }
            }
        } else if( action == down ) {
            if( new_coord.col_ != coord.col_ ) {
                p = 0;
            } else {
                if( coord.row_ > 0 ) {
                    p = new_coord.row_ + 1 == coord.row_ ? q : (new_coord.row_ == coord.row_ ? 1 - q : 0);
                } else {
                    assert(coord.row_ == 0);
                    p = new_coord.row_ == coord.row_ ? 1 : 0;
                }
            }
        } else if( action == left ) {
            if( new_coord.row_ != coord.row_ ) {
                p = 0;
            } else {
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
    float probability_tr_loc_oreslam(int action, int old_loc, int new_loc, float q) const {
        return probability_tr_loc_standard(action, old_loc, new_loc, q);
    }

  public:
    float probability_tr_loc(int action, int old_loc, int new_loc, float pa = -1) const {
        float q = pa == -1 ? pa_ : pa;
        return !oreslam_ ? probability_tr_loc_standard(action, old_loc, new_loc, q)
                         : probability_tr_loc_oreslam(action, old_loc, new_loc, q);
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
            if( drand48() < q ) {
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
    int sample_loc_oreslam(int loc, int action, float q) const {
        return sample_loc_standard(loc, action, q);
    }

  public:
    int sample_loc(int loc, int action, float pa = -1) const {
        float q = pa == -1 ? pa_ : pa;
        return !oreslam_ ? sample_loc_standard(loc, action, q)
                         : sample_loc_oreslam(loc, action, q);
    }

    // model for observations.
    //
    // For the non-ore-slam case, P( obs | label at current loc is 'label' ) is equal to
    // parameter po if obs = label and (1 - po) / (nlabels - 1) otherwise.

    // computes P(obs | label at current loc is 'label')
    float probability_obs_standard(int obs, int /*loc*/, int label, int /*last_action*/) const {
        return label == obs ? po_ : (1 - po_) / float(nlabels_ - 1);
    }

    // computes P(obs | loc, map given by labels)
    float probability_obs_standard(int obs, int loc, const std::vector<int> &labels, int last_action) const {
        return probability_obs_standard(obs, loc, labels[loc], last_action);
    }

  private:
    int sample_obs_standard(int /*loc*/, int label, int /*last_action*/, float q) const {
        assert((q >= 0) && (q <= 1));
        assert(!oreslam_);
        if( drand48() >= q ) {
            int i = lrand48() % (nlabels_ - 1);
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
    int sample_obs_standard(int loc, int last_action, float q) const {
        assert((loc >= 0) && (loc < nloc_));
        int label = cells_[loc].label_;
        return sample_obs_standard(loc, label, last_action, q);
    }

    int sample_obs_oreslam(int loc, int /*last_action*/, float q) const {
        assert((q >= 0) && (q <= 1));
        assert(oreslam_);
        assert((loc >= 0) && (loc < nloc_));
        int obs = 0;
        int row = loc / ncols_, col = loc % ncols_;
        for( int dr = -1; dr < 2; ++dr ) {
            int nrow = row + dr;
            if( (nrow < 0) || (nrow >= nrows_) ) continue; // for outside-of-the-grid cell, sampled value = 0
            for( int dc = -1; dc < 2; ++dc ) {
                int ncol = col + dc;
                if( (ncol < 0) || (ncol >= ncols_) ) continue; // for outside-of-the-grid cell, sampled value = 0
                int new_loc = nrow * ncols_ + ncol;
                int label = cells_[new_loc].label_;
                assert((label == 0) || (label == 1));
                float p = base_obs_noise_ * (dr != 0 ? q : 1) * (dc != 0 ? q : 1);
                obs += drand48() < p ? label : 1 - label;
            }
        }
        assert((obs >= 0) && (obs < 10));
        return obs;
    }

  public:
    int sample_obs(int loc, int last_action, float po = -1) const {
        float q = po == -1 ? po_ : po;
        return !oreslam_ ? sample_obs_standard(loc, last_action, q)
                         : sample_obs_oreslam(loc, last_action, q);
    }

    // For the ore-slam case, observation is a number from 0 to 9 that is computed from
    // the current location and the 8 surrounding locations (for a middle location).
    // The model is the following:
    //
    //     obs = \sum_loc I(loc)
    //
    // where the sum is over all locations loc surrounding (and including) the current
    // location cloc. The variable I(loc) is a indicator random variable that is equal
    // to label[loc] with probability equal to po^dist(loc,cloc) where dist(loc,cloc)
    // is the Manhattan distance between loc and cloc.

  public:
    float probability_obs_oreslam(int obs, int loc, int slabels, int /*last_action*/) const {
        assert(oreslam_);
        return probability_obs_oreslam_[calculate_index(slabels, obs, loc_type_[loc])];
    }

    float probability_obs_oreslam(int obs, int loc, const std::vector<int> &labels, int last_action) const {
        assert(oreslam_);
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
        return probability_obs_oreslam(obs, loc, slabels, last_action);
    }

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

    void precompute_stored_information_oreslam() {
        assert(oreslam_);

        // num bits equal to 1 for each integer in 0..511
        num_bits_ = std::vector<int>(512, 0);
        for( int n = 0; n < 512; ++n ) {
            for( int j = 0; j < 9; ++j )
                num_bits_[n] += bit(n, j);
        }

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

        // dimension is product of 512 valuations, 10 obs and 9 loc-types
        probability_obs_oreslam_ = std::vector<float>(512 * 10 * 9, 0);
        for( int loc_type = 0; loc_type < 9; ++loc_type ) {
            for( int obs = 0; obs < 10; ++obs ) {
                for( int slabels = 0; slabels < 512; ++slabels ) {
                    //if( incompatible_slabels(slabels, loc_type) ) continue;
                    int index = calculate_index(slabels, obs, loc_type);
                    for( int valuation = 0; valuation < 512; ++valuation ) {
                        //if( incompatible_slabels(valuation, loc_type) ) continue;
                        if( num_bits_[valuation] != obs ) continue;
                        float p = 1;
                        for( int rloc = 0; rloc < 9; ++rloc ) { // computes \prod_{loc} P( val[loc] | slabels[loc] ) onto p
                            if( incompatible_loc(rloc, loc_type) ) {
                                //continue;
                                p *= bit(valuation, rloc) == 0 ? 0.99 : 0.01;
                            } else {
                                // loc=4 refers to the center of the 3x3 window
                                float q = base_obs_noise_ * (same_column(rloc, 4, 3) ? 1 : po_) * (same_row(rloc, 4, 3) ? 1 : po_);
                                p *= bit(valuation, rloc) == bit(slabels, rloc) ? q : 1 - q;
                            }
                        }
                        probability_obs_oreslam_[index] += p;
                    }
                    assert(probability_obs_oreslam_[index] <= 1);
                    if( probability_obs_oreslam_[index] == 0 ) {
                        std::cout << "error: prob(obs=" << obs
                                  << "|slabels=" << slabels
                                  << ", loc-type=" << loc_type
                                  << ") = " << probability_obs_oreslam_[index]
                                  << std::endl;
                        assert(probability_obs_oreslam_[index] > 0);
                    }
                }
            }
        }

        for( int obs = 0; obs < 10; ++obs ) {
            float p = probability_obs_oreslam_[calculate_index(0, obs, LOC_MIDDLE)];
            std::cout << "# P(obs=" << obs << "|slabels=0,MIDDLE)=" << p << std::endl;
        }
        for( int obs = 0; obs < 10; ++obs ) {
            float p = probability_obs_oreslam_[calculate_index(0, obs, LOC_CORNER_UP_RI)];
            std::cout << "# P(obs=" << obs << "|slabels=0,CORNER)=" << p << std::endl;
        }
        for( int obs = 0; obs < 10; ++obs ) {
            float p = probability_obs_oreslam_[calculate_index(0, obs, LOC_EDGE_UP)];
            std::cout << "# P(obs=" << obs << "|slabels=0,EDGE)=" << p << std::endl;
        }

        // check probabilities for observation: \sum_{obs} P( obs | slabels ) = 1 for all slabels
        for( int loc_type = 0; loc_type < 9; ++loc_type ) {
            for( int slabels = 0; slabels < 512; ++slabels ) {
                //if( incompatible_slabels(slabels, loc_type) ) continue;
                float sum = 0;
                for( int obs = 0; obs < 10; ++obs )
                    sum += probability_obs_oreslam_[calculate_index(slabels, obs, loc_type)];
                assert(fabs(sum - 1) < 1e-6);
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
                    float p = probability_obs_oreslam_[calculate_index(slabels, obs, loc_type)];
                    if( p > best_obs_prob ) {
                        best_obs = obs;
                        best_obs_prob = p;
                    }
                }
                int correct_obs = num_bits(slabels);
                if( best_obs != correct_obs ) {
                    std::cout << "warning: most plausible obs for (loc-type=" << loc_type
                              << ", slabels=" << slabels
                              << ", bits=|";
                    print_bits(std::cout, slabels, 9);
                    std::cout << "|) is " << best_obs
                              << " with p=" << best_obs_prob
                              << "; correct obs " << correct_obs
                              << " has p="
                              << probability_obs_oreslam_[calculate_index(slabels, correct_obs, loc_type)]
                              << std::endl;
                }
            }
        }
#endif

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

    int num_bits(int valuation) const {
        assert((valuation >= 0) && (valuation < 512));
        return num_bits_[valuation];
    }

    int var_offset(int loc, int var_id) const {
        assert((loc >= 0) && (loc < nloc_));
        assert((var_id >= 0) && (var_id < nloc_));
        return var_offset_[loc * nloc_ + var_id];
    }


    // tracking function

    void initialize(std::vector<tracking_t<cellmap_t>*> &tracking_algorithms, std::vector<repository_t> &repos) const {
        repos.clear();
        repos.reserve(tracking_algorithms.size());
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            tracking_t<cellmap_t> &tracking = *tracking_algorithms[i];
            tracking.initialize();
            repos.push_back(repository_t());
        }
    }
    void advance_step(std::vector<repository_t> &repos, int last_action, int obs, std::vector<tracking_t<cellmap_t>*> &tracking_algorithms, bool print_marginals = false) const {
        assert(repos.size() == tracking_algorithms.size());
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            tracking_t<cellmap_t> &tracking = *tracking_algorithms[i];
            tracking.update(last_action, obs);
            tracking.calculate_marginals();
            tracking.store_marginals(repos[i]);
            if( print_marginals ) tracking.print_marginals(std::cout, repos[i]);
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
            os << "[" << coord_t(loc_) << "," << obs_ << "," << last_action_ << "]";
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
    };

    int random_action() const {
        int index = lrand48();
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
                       std::vector<tracking_t<cellmap_t>*> &tracking_algorithms,
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
        initialize(tracking_algorithms, repos);
        output_execution.push_back(initial_step);
        advance_step(repos, -1, obs, tracking_algorithms, print_marginals);
        std::cout << std::endl;

        // run execution
        std::cout << "# steps:";
        for( size_t t = 1; true; ++t ) {
            std::cout << std::endl << " #" << t << std::flush;
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
                assert(!tracking_algorithms.empty());
                last_action = policy->select_action(tracking_algorithms[0]);
                if( last_action == -1 ) break;
                hidden_loc = sample_loc(hidden_loc, last_action);
                obs = sample_obs(hidden_loc, last_action);
                std::cout << "[hidden-loc=" << coord_t(hidden_loc) << ",obs=" << obs << "]";
            }

            // update tracking
            output_execution.push_back(execution_step_t(hidden_loc, obs, last_action));
            advance_step(repos, last_action, obs, tracking_algorithms, print_marginals);
        }
        std::cout << std::endl;
    }
    void run_execution(std::vector<repository_t> &repos,
                       const execution_t &input_execution,
                       execution_t &output_execution,
                       std::vector<tracking_t<cellmap_t>*> &tracking_algorithms,
                       bool print_marginals = false) {
        run_execution(repos, input_execution, output_execution, 0, 0, tracking_algorithms, print_marginals);
    }
    void run_execution(std::vector<repository_t> &repos,
                       execution_t &output_execution,
                       int initial_loc,
                       int nsteps,
                       const action_selection_t<cellmap_t> &policy,
                       std::vector<tracking_t<cellmap_t>*> &tracking_algorithms,
                       bool print_marginals = false) {
        execution_step_t initial_step(initial_loc, -1, -1);
        execution_t empty_execution(1, initial_step);
        run_execution(repos, empty_execution, output_execution, nsteps, &policy, tracking_algorithms, print_marginals);
    }

    // scoring of tracking algorithms
    typedef int score_t;

    score_t compute_score(int current_loc, const std::set<int> &relevant_cells, const tracking_t<cellmap_t> &tracking) const {
        score_t score = 0;
        for( int var = 0; var < 1 + nrows_ * ncols_; ++var ) {
            if( relevant_cells.find(var) != relevant_cells.end() ) {
                dai::Factor marginal;
                tracking.get_marginal(var, marginal);
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
    score_t compute_score(const execution_t &execution, const tracking_t<cellmap_t> &tracking) const {
        std::set<int> relevant_cells;
        for( size_t i = 0; i < execution.size(); ++i )
            relevant_cells.insert(execution[i].loc_);
        int current_loc = execution.back().loc_;
        return compute_score(current_loc, relevant_cells, tracking);
    }

    void compute_scores(const execution_t &execution, const std::vector<tracking_t<cellmap_t>*> &tracking_algorithms, std::vector<score_t> &scores) const {
        scores.clear();
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            scores.push_back(compute_score(execution, *tracking_algorithms[i]));
    }

    // R plots
    void generate_R_plot(std::ostream &os, const tracking_t<cellmap_t> &tracking, const repository_t &repository) const {
        for( size_t t = 0; t < repository.size(); ++t ) {
            const float *loc_marginal = tracking.stored_marginal(repository, t, nloc_);
            os << "mar_" << tracking.name_ << "_t" << t << " <- c(";
            for( int value = 0; value < nloc_; ++value ) {
                os << loc_marginal[value];
                if( 1 + value < nloc_ ) os << ", ";
            }
            os << ");" << std::endl;
        }
        os << "mar_" << tracking.name_ << " <- c(";
        for( size_t t = 0; t < repository.size(); ++t )
            os << "mar_" << tracking.name_ << "_t" << t << (t + 1 < repository.size() ? ", " : "");
        os << ");" << std::endl;

        int ncols_in_plot = nrows_ * ncols_;
        os << "dmf <- melt(as.data.frame(t(matrix(mar_" << tracking.name_ << ", ncol=" << ncols_in_plot << ", byrow=T))));" << std::endl
           << "dmf$y <- rep(1:" << ncols_in_plot << ", " << repository.size() << ");" << std::endl
           << "plot_" << tracking.name_ << " <- ggplot(dmf,aes(y=variable, x=y)) + "
           << "geom_tile(aes(fill=value)) + "
           //<< "geom_text(aes(label=ifelse(value>.01, trunc(100*value, digits=2)/100, \"\")), size=3, angle=0, color=\"white\") +"
           << "labs(y=\"time step\", x=\"location\") + "
           << "theme_minimal() + "
           //<< "scale_x_discrete(limits=1:" << ncols_in_plot << ") + "
           //<< "scale_y_discrete(labels=paste(\"t=\", 0:" << tracking.marginals_.size() << ", sep=\"\"));"
           //<< "scale_y_discrete(labels=rep(\"\", " << tracking.marginals_.size() << "));"
           << std::endl;

        // save in plot in pdf
        os << "pdf(\"marginals_" << tracking.name_ << ".pdf\");" << std::endl
           << "show(plot_" << tracking.name_ << ")" << std::endl
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

