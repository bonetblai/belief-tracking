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

#ifndef SLAM2_PARTICLE_TYPES_H
#define SLAM2_PARTICLE_TYPES_H

#include <cassert>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <stdlib.h>

#include <dai/alldai.h>

#include "cellmap.h"
#include "slam_particle_types.h"
#include "utils.h"

//#define DEBUG

// Abstract Particle for the 2nd Rao-Blackwellised filter
struct rbpf_slam2_particle_t : public base_particle_t {
    std::vector<int> loc_history_;

    // variables, factors, and centers
    std::vector<dai::Var> variables_;
    std::vector<dai::Factor> factors_;
    std::vector<int> centers_;  // not clear for what?

    // computation of marginal in factor model
    mutable bool need_to_recalculate_marginals_;
    mutable std::vector<dai::Factor> marginals_;
    mutable dai::FactorGraph jt_factor_graph_;
    mutable dai::JTree jt_;

    rbpf_slam2_particle_t() {
        assert(base_ != 0);
        assert(base_->nlabels_ == 2);

        // create binary variables for each cell in the grid
        int nloc = base_->nloc_;
        variables_ = std::vector<dai::Var>(nloc);
        for( int loc = 0; loc < nloc; ++loc ) {
            variables_[loc] = dai::Var(loc, 2);
            //std::cout << "loc=" << loc << ", coord=" << coord_t(loc) << std::endl;
        }

        // create one factor for each location. The variables
        // in the factor are the variables for the location
        // surrounding the factor, including the variable
        // for the "center" location. Also set up the center
        // for each factor.
        centers_ = std::vector<int>(nloc);
        factors_ = std::vector<dai::Factor>(nloc);
        marginals_ = std::vector<dai::Factor>(nloc);
        for( int loc = 0; loc < nloc; ++loc ) {
            int row = loc / base_->ncols_, col = loc % base_->ncols_;
            std::vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= base_->nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= base_->ncols_) ) continue;
                    if( (dr == 0) && (dc == 0) ) centers_[loc] = vars.size();
                    vars.push_back(variables_[nr * base_->ncols_ + nc]);
                }
            }
            std::sort(vars.begin(), vars.end());
            dai::VarSet varset(vars.begin(), vars.end());
            factors_[loc] = dai::Factor(varset);
            marginals_[loc] = dai::Factor(varset);
        }

#ifdef DEBUG
        for( int loc = 0; loc < nloc; ++loc )
            ;//print_factor(std::cout, loc, factors_[loc], "factors_");
#endif

        // computation of marginal
        need_to_recalculate_marginals_ = true;
    }
    virtual ~rbpf_slam2_particle_t() { }

    void print_factor(std::ostream &os, int loc, const dai::Factor &factor, const std::string &name) const {
        os << "variables[loc=" << loc << "]=";
        for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it )
            os << " " << *it;
        os << std::endl << name << "[loc=" << loc << "]=" << std::endl;
        for( int j = 0; j < int(factor.nrStates()); ++j ) {
            os << "   ";
            std::map<dai::Var, size_t> states = dai::calcState(factor.vars(), j);
            for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it )
                os << " " << std::setw(2) << *it << "=" << states[*it];
            os << ":  j=" << std::setw(3) << j << ",  nbits=" << base_->num_bits(j) << ",  value=" << factor[j] << std::endl;
        }
    }

    void initial_sampling() {
        assert(base_->nlabels_ == 2);

        // set initial history and reset factors for locations
        loc_history_.push_back(base_->initial_loc_);
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / (1 << factor.vars().size());
            for( int i = 0; i < (1 << factor.vars().size()); ++i )
                factor.set(i, p);
            //std::cout << "factor-size: loc=" << loc << ":" << coord_t(loc) << ", sz=" << factor.vars().size() << std::endl;
        }
        calculate_marginals();
    }

    int get_slabels(int loc, int value) const {
        //std::cout << "get_slabels(loc=" << loc << ", value=" << value << ")=" << std::flush;
        int slabels = 0, i = 0;
        for( dai::VarSet::const_iterator it = factors_[loc].vars().begin(); it != factors_[loc].vars().end(); ++it, ++i ) {
            if( value & 0x1 ) {
                int var_id = it->label();
                int var_off = base_->var_offset(loc, var_id);
                //std::cout << "[var_id=" << var_id << ", off=" << var_off << "]";
                slabels += (1 << var_off);
            }
            value = value >> 1;
        }
        //print_bits(std::cout, slabels, 9);
        //std::cout << " (" << slabels << ")" << std::endl;
        return slabels;
    }

    void update_factors(int last_action, int obs) {
        int current_loc = loc_history_.back();
#ifdef DEBUG
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        print_factor(std::cout, current_loc, factors_[current_loc], "factors_");
#endif
        assert(current_loc < int(factors_.size()));
        dai::Factor &factor = factors_[current_loc];
        float total_mass = 0.0;
        for( int j = 0; j < int(factor.nrStates()); ++j ) {
            //std::cout << "case j=" << std::setw(3) << j << std::endl;
            int slabels = get_slabels(current_loc, j);
            factor.set(j, factor[j] * base_->probability_obs_special(obs, current_loc, slabels, last_action));
            total_mass += factor[j];
        }
std::cout << "loc=" << current_loc << ", coord=" << coord_t(current_loc) << ", obs=" << obs << std::endl;
        assert(total_mass > 0);
        factor /= total_mass;
        need_to_recalculate_marginals_ = true;
#ifdef DEBUG
        std::cout << "factor after  update: loc=" << current_loc << ", obs=" << obs << std::endl;
        print_factor(std::cout, current_loc, factors_[current_loc], "factors_");
#endif
    }

    void calculate_marginals() const {
        if( need_to_recalculate_marginals_ ) {
            //CHECK std::cout << "junction tree: BEGIN" << std::endl;
            apply_junction_tree();
            extract_marginals_from_junction_tree();
            need_to_recalculate_marginals_ = false;
            //CHECK std::cout << "junction tree: END" << std::endl;
        }
    }

    void update_marginal(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        //CHECK std::cout << "HOLA: weight=" << weight << std::endl;
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            //CHECK print_factor(std::cout, loc, marginals_[loc], "marginals_");
            dai::Factor marginal = marginals_[loc].marginal(variables_[loc]);
            for( int label = 0; label < base_->nlabels_; ++label ) {
                marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * marginal[label]);
                //CHECK std::cout << "marginal[loc=" << loc << ", label=" << label << "]=" << marginal[label] << std::endl;
            }
        }
        int current_loc = loc_history_.back();
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
    }

    int value_for(int /*var*/) const { return -1; }

    virtual void sample_from_pi(rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const = 0;
    virtual float importance_weight(const rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const = 0;


    void apply_junction_tree() const {
        dai::PropertySet opts;
        jt_factor_graph_ = dai::FactorGraph(factors_);
        size_t maxstates = 1e6;

        // compute junction tree using min-fill heuristic
        try {
            dai::boundTreewidth(jt_factor_graph_, &dai::eliminationCost_MinFill, maxstates);
        } catch( dai::Exception &e ) {
            if( e.getCode() == dai::Exception::OUT_OF_MEMORY )
                std::cout << "error: cannot compute junction tree (need more than " << maxstates << " states)" << std::endl;
        }

        // run junction tree
        jt_ = dai::JTree(jt_factor_graph_, opts("updates", std::string("HUGIN")));
        jt_.init();
        jt_.run();
    }

    void extract_marginals_from_junction_tree() const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            marginals_[loc] = jt_.belief(factors_[loc].vars());
        }
    }
};

// Particle for the motion model RBPF filter (slam2)
struct motion_model_rbpf_slam2_particle_t : public rbpf_slam2_particle_t {
    virtual void sample_from_pi(rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        np = p;
        int next_loc = base_->sample_loc(p.loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        np.update_factors(last_action, obs);
        np.calculate_marginals();
    }

    virtual float importance_weight(const rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        assert(!p.need_to_recalculate_marginals_);
        float weight = 0;
        int np_current_loc = np.loc_history_.back();
        const dai::Factor &p_marginal = p.marginals_[np_current_loc];
        for( int slabels = 0; slabels < int(p_marginal.nrStates()); ++slabels )
            weight += p_marginal[slabels] * base_->probability_obs_special(obs, np_current_loc, slabels, last_action);
        return weight;
    }
};

#if 0 // optimal RBPF filter
// Particle for the optimal RBPF filter (verified: 09/12/2015)
struct optimal_rbpf_slam2_particle_t : public rbpf_slam2_particle_t {
    mutable std::vector<float> cdf_;

    void calculate_cdf(const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        cdf_.clear();
        cdf_.reserve(base_->nloc_);

        int current_loc = p.loc_history_.back();
        float previous = 0;

        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            float prob = 0;
            for( int label = 0; label < base_->nlabels_; ++label )
                prob += base_->probability_obs(obs, label, last_action) * p.probability(label, new_loc);
            cdf_.push_back(previous + base_->probability_tr_loc(last_action, current_loc, new_loc) * prob);
            previous = cdf_.back();
        }

        // normalize
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            cdf_[new_loc] /= cdf_.back();
        }
        assert(cdf_.back() == 1.0);
    }

    virtual void sample_from_pi(rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        // sample new_loc w.p. P(new_loc|curr_loc,last_action) * SUM_c P(obs|new_loc,Label[new_loc]=c) P(c|new_loc)
        np = p;
        calculate_cdf(p, last_action, obs);
        int next_loc = Utils::sample_from_distribution(base_->nloc_, &cdf_[0]);
        np.loc_history_.push_back(next_loc);
        np.update_factors(last_action, obs);
    }

    virtual float importance_weight(const rbpf_slam2_particle_t &/*np*/, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        int current_loc = p.loc_history_.back();
        float weight = 0;
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            float prob = 0;
            for( int label = 0; label < base_->nlabels_; ++label )
                prob += base_->probability_obs(obs, label, last_action) * p.probability(label, new_loc);
            weight += base_->probability_tr_loc(last_action, current_loc, new_loc) * prob;
        }
        return weight;
    }
};
#endif

#undef DEBUG

#endif

