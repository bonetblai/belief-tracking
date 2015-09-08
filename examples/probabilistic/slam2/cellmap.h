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


struct coord_t {
    int col_;
    int row_;
    static int ncols_;

    coord_t(int col, int row) : col_(col), row_(row) { }
    coord_t(int loc) : col_(loc % ncols_), row_(loc / ncols_) { }
    int as_index() const { return row_ * ncols_ + col_; }
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

    float pa_;
    float po_;
    float pc_;

    std::vector<cell_t> cells_;

    mutable int initial_loc_;

    cellmap_t(int nrows, int ncols, int nlabels, float pa, float po, float pc)
      : nrows_(nrows), ncols_(ncols), nloc_(nrows * ncols), nvars_(1 + nloc_),
        nlabels_(nlabels), pa_(pa), po_(po), pc_(pc) {
        cells_ = std::vector<cell_t>(nloc_);
    }
    ~cellmap_t() { }

    void set_labels(const int *labels, size_t size) {
        assert(size == cells_.size());
        for( size_t i = 0; i < size; ++i )
            cells_[i].label_ = labels[i];
    }
    void set_labels(const std::vector<int> labels) {
        set_labels(&labels[0], labels.size());
    }

    void sample_labels() {
        std::vector<int> labels(cells_.size(), 0);
        for( size_t i = 0; i < labels.size(); ++i )
            labels[i] = lrand48() % nlabels_;
        set_labels(labels);
    }

    // API for particle filters
    int domain_size(int var) const {
        return var == nvars_ - 1 ? nloc_ : nlabels_;
    }

    // action labels
    enum { up, right, down, left };

    // computes P(new_loc | old_lod, action)
    float loc_probability(int action, int old_loc, int new_loc) const {
        float p = 0;
        coord_t coord(old_loc), new_coord(new_loc);
        if( action == up ) {
            if( coord.row_ + 1 < nrows_ ) {
                p = new_coord.row_ == coord.row_ + 1 ? pa_ : (new_coord.row_ == coord.row_ ? 1 - pa_ : 0);
            } else {
                assert(coord.row_ + 1 == nrows_);
                p = new_coord.row_ == coord.row_ ? 1 : 0;
            }
        } else if( action == right ) {
            if( coord.col_ + 1 < ncols_ ) {
                p = new_coord.col_ == coord.col_ + 1 ? pa_ : (new_coord.col_ == coord.col_ ? 1 - pa_ : 0);
            } else {
                assert(coord.col_ + 1 == ncols_);
                p = new_coord.col_ == coord.col_ ? 1 : 0;
            }
        } else if( action == down ) {
            if( coord.row_ > 0 ) {
                p = new_coord.row_ + 1 == coord.row_ ? pa_ : (new_coord.row_ == coord.row_ ? 1 - pa_ : 0);
            } else {
                assert(coord.row_ == 0);
                p = new_coord.row_ == coord.row_ ? 1 : 0;
            }
        } else if( action == left ) {
            if( coord.col_ > 0 ) {
                p = new_coord.col_ + 1 == coord.col_ ? pa_ : (new_coord.col_ == coord.col_ ? 1 - pa_ : 0);
            } else {
                assert(coord.col_ == 0);
                p = new_coord.col_ == coord.col_ ? 1 : 0;
            }
        }
        return p;
    }

    // computes P(obs | label at current loc is 'label')
    float obs_probability(int obs, int label, int /*last_action*/) const {
        return label == obs ? po_ : (1 - po_) / float(nlabels_ - 1);
    }

    // computes P(obs | loc, map given by labels)
    float obs_probability(int obs, int loc, const std::vector<int> &labels, int last_action) const {
        return obs_probability(obs, labels[loc], last_action);
    }

    // run execution: action application and observation sampling
    int regress_loc(int new_loc, int action) const {
        coord_t coord(new_loc), new_coord(new_loc);
        if( action == up )
            coord = coord_t(new_coord.col_, new_coord.row_ == 0 ? 0 : new_coord.row_ - 1);
        else if( action == right )
            coord = coord_t(new_coord.col_ == 0 ? 0 : new_coord.col_ - 1, new_coord.row_);
        else if( action == down )
            coord = coord_t(new_coord.col_, new_coord.row_ == nrows_ - 1 ? nrows_ - 1 : new_coord.row_ + 1);
        else if( action == left )
            coord = coord_t(new_coord.col_ == ncols_ - 1 ? ncols_ - 1 : new_coord.col_ + 1, new_coord.row_);
        return coord.as_index();
    }

    int sample_loc(int loc, int action) const {
        coord_t coord(loc), new_coord(loc);
        if( drand48() < pa_ ) {
            if( action == up )
                new_coord = coord_t(coord.col_, coord.row_ + 1 < nrows_ ? coord.row_ + 1 : coord.row_);
            else if( action == right )
                new_coord = coord_t(coord.col_ + 1 < ncols_ ? coord.col_ + 1 : coord.col_, coord.row_);
            else if( action == down )
                new_coord = coord_t(coord.col_, coord.row_ > 0 ? coord.row_ - 1 : coord.row_);
            else if( action == left )
                new_coord = coord_t(coord.col_ > 0 ? coord.col_ - 1 : coord.col_, coord.row_);
        }
        return new_coord.as_index();
    }

    int sample_obs(int loc, int /*last_action*/) const {
        assert((loc >= 0) && (loc < int(cells_.size())));
        int label = cells_[loc].label_;
        if( drand48() > po_ ) {
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

    void initialize(std::vector<tracking_t<cellmap_t>*> &tracking_algorithms) const {
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            tracking_t<cellmap_t> &tracking = *tracking_algorithms[i];
            tracking.initialize();
            tracking.calculate_marginals();
            tracking.store_marginals();
        }
    }
    void advance_step(int last_action, int obs, std::vector<tracking_t<cellmap_t>*> &tracking_algorithms) const {
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            tracking_t<cellmap_t> &tracking = *tracking_algorithms[i];
            tracking.update(last_action, obs);
            tracking.calculate_marginals();
            tracking.store_marginals();
        }
    }

    struct execution_step_t {
        int loc_;
        int obs_;
        int last_action_;
        execution_step_t(int loc = 0, int obs = 0, int last_action = 0)
          : loc_(loc), obs_(obs), last_action_(last_action) { }
    };
    struct execution_t : public std::vector<execution_step_t> { };

    void run_execution(const int *labels,
                       int labels_size,
                       const execution_t &input_execution,
                       execution_t &output_execution,
                       int nsteps,
                       const action_selection_t<cellmap_t> *policy,
                       std::vector<tracking_t<cellmap_t>*> &tracking_algorithms) {
        // set labels for callmap
        if( labels_size > 0 )
            set_labels(labels, labels_size);
        else
            sample_labels();

        // initialize tracking algorithms and output execution
        initialize(tracking_algorithms);
        output_execution.clear();
        output_execution.push_back(execution_step_t(0, -1, -1));

        int hidden_loc = 0;
        for( size_t t = 0; true; ++t ) {
            if( !input_execution.empty() && (t >= input_execution.size()) ) return;
            if( input_execution.empty() && (t >= size_t(nsteps)) ) return;
            int last_action = -1;
            int obs = -1;

            // select next action, update hidden loc, and sample observation
            if( !input_execution.empty() ) {
                hidden_loc = input_execution[t].loc_;
                obs = input_execution[t].obs_;
                last_action = input_execution[t].last_action_;
            } else {
                assert(policy != 0);
                assert(!tracking_algorithms.empty());
                last_action = policy->select_action(tracking_algorithms[0]);
                hidden_loc = sample_loc(hidden_loc, last_action);
                obs = sample_obs(hidden_loc, last_action);
                std::cerr << "step (t=" << t << "): last_action=" << last_action << ", obs=" << obs << ", loc=" << hidden_loc << std::endl;
            } 

            // update tracking
            output_execution.push_back(execution_step_t(hidden_loc, obs, last_action));
            advance_step(last_action, obs, tracking_algorithms);
        }
    }
    void run_execution(const int *labels,
                       int labels_size,
                       const execution_t &input_execution,
                       execution_t &output_execution,
                       std::vector<tracking_t<cellmap_t>*> &tracking_algorithms) {
        run_execution(labels, labels_size, input_execution, output_execution, 0, 0, tracking_algorithms);
    }
    void run_execution(const int *labels,
                       int labels_size,
                       execution_t &output_execution,
                       int nsteps,
                       const action_selection_t<cellmap_t> &policy,
                       std::vector<tracking_t<cellmap_t>*> &tracking_algorithms) {
        execution_t empty_execution;
        run_execution(labels, labels_size, empty_execution, output_execution, nsteps, &policy, tracking_algorithms);
    }
    void run_execution(const std::vector<int> &labels,
                       const execution_t &input_execution,
                       execution_t &output_execution,
                       std::vector<tracking_t<cellmap_t>*> &tracking_algorithms) {
        run_execution(&labels[0], labels.size(), input_execution, output_execution, 0, 0, tracking_algorithms);
    }
    void run_execution(const std::vector<int> &labels,
                       execution_t &output_execution,
                       int nsteps,
                       const action_selection_t<cellmap_t> &policy,
                       std::vector<tracking_t<cellmap_t>*> &tracking_algorithms) {
        execution_t empty_execution;
        run_execution(&labels[0], labels.size(), empty_execution, output_execution, nsteps, &policy, tracking_algorithms);
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
    void generate_R_plot(std::ostream &os, const tracking_t<cellmap_t> &tracking) const {
        for( size_t t = 0; t < tracking.marginals_.size(); ++t ) {
            os << "mar_" << tracking.name_ << "_t" << t << " <- c"
               << tracking.stored_marginal(t, nrows_ * ncols_).p()
               << ";" << std::endl;
        }
        os << "mar_" << tracking.name_ << " <- c(";
        for( size_t t = 0; t < tracking.marginals_.size(); ++t )
            os << "mar_" << tracking.name_ << "_t" << t << (t + 1 < tracking.marginals_.size() ? ", " : "");
        os << ");" << std::endl;

        os << "dmf <- melt(as.data.frame(t(matrix(mar_" << tracking.name_ << ", ncol=" << ncols_ << ", byrow=T))));" << std::endl
           << "dmf$y <- rep(1:" << ncols_ << ", " << tracking.marginals_.size() << ");" << std::endl
           << "plot_" << tracking.name_ << " <- ggplot(dmf,aes(y=variable, x=y)) + "
           << "geom_tile(aes(fill=value)) + "
           << "geom_text(aes(label=ifelse(value>.01, trunc(100*value, digits=2)/100, \"\")), size=3, angle=0, color=\"white\") +"
           << "labs(y=\"time step\", x=\"location\") + "
           << "theme_minimal() + "
           << "scale_x_discrete(limits=1:" << ncols_ << ") + "
           << "scale_y_discrete(labels=paste(\"t=\", 0:" << tracking.marginals_.size() << ", sep=\"\"));" << std::endl;

        os << "pdf(\"marginals_" << tracking.name_ << ".pdf\");" << std::endl
           << "show(plot_" << tracking.name_ << ")" << std::endl
           << "dev.off();" << std::endl;
    }
};

#endif

