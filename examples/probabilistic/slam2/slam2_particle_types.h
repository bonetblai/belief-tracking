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
#include <cstdlib>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>

#include <dai/alldai.h>

#include "cellmap.h"
#include "slam_particle_types.h"
#include "utils.h"

//#define DEBUG

// Abstract Particle for the 2nd Rao-Blackwellised filter
struct rbpf_slam2_particle_t : public base_particle_t {
    std::vector<int> loc_history_;

    // variables and factors
    std::vector<dai::Factor> factors_;

    // computation of marginal in factor model
    mutable bool need_to_recalculate_marginals_;
    mutable std::vector<dai::Factor> marginals_;
    dai::InfAlg *inference_algorithm_;

    rbpf_slam2_particle_t() : inference_algorithm_(0) {
        assert(base_ != 0);
        assert(base_->nlabels_ == 2);

        // create binary variables for each cell in the grid
        int nloc = base_->nloc_;
        std::vector<dai::Var> variables(nloc);
        for( int loc = 0; loc < nloc; ++loc ) {
            variables[loc] = dai::Var(loc, 2);
            //std::cout << "loc=" << loc << ", coord=" << coord_t(loc) << std::endl;
        }

        // create one factor for each location. The variables
        // in the factor are the variables for the location
        // surrounding the factor, including the variable
        // for the "center" location. Also set up the center
        // for each factor.
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
                    vars.push_back(variables[nr * base_->ncols_ + nc]);
                }
            }
            std::sort(vars.begin(), vars.end());
            dai::VarSet varset(vars.begin(), vars.end());
            factors_[loc] = dai::Factor(varset);
            marginals_[loc] = dai::Factor(varset);
        }

#ifdef DEBUG
        for( int loc = 0; loc < nloc; ++loc )
            print_factor(std::cout, loc, factors_[loc], "factors_");
#endif

        // computation of marginal
        need_to_recalculate_marginals_ = true;

        // create algorithm
        create_and_initialize_inference_algorithm();
    }
    virtual ~rbpf_slam2_particle_t() {
        destroy_inference_algorithm();
    }

    rbpf_slam2_particle_t(const rbpf_slam2_particle_t &p) {
        *this = p;
    }

    rbpf_slam2_particle_t(rbpf_slam2_particle_t &&p)
      : loc_history_(p.loc_history_), factors_(p.factors_),
        need_to_recalculate_marginals_(p.need_to_recalculate_marginals_),
        marginals_(p.marginals_) {
        inference_algorithm_ = p.inference_algorithm_;
        p.inference_algorithm_ = 0;
    }

    const rbpf_slam2_particle_t& operator=(const rbpf_slam2_particle_t &p) {
        loc_history_ = p.loc_history_;
        factors_ = p.factors_;
        need_to_recalculate_marginals_ = p.need_to_recalculate_marginals_;
        marginals_ = p.marginals_;
        if( p.inference_algorithm_ != 0 )
            inference_algorithm_ = p.inference_algorithm_->clone();
        else
            inference_algorithm_ = 0;
        return *this;
    }

    bool operator==(const rbpf_slam2_particle_t &p) const {
        return (loc_history_ == p.loc_history_) && (factors_ == p.factors_) &&
               (need_to_recalculate_marginals_ == p.need_to_recalculate_marginals_) &&
               (marginals_ == p.marginals_) &&
               (((inference_algorithm_ == 0) && (p.inference_algorithm_ == 0)) ||
                ((inference_algorithm_ != 0) && (p.inference_algorithm_ != 0)));
    }

    void create_and_initialize_inference_algorithm() {
        dai::FactorGraph factor_graph(factors_);

#if 0
        // compute junction tree using min-fill heuristic
        size_t maxstates = 1e6;
        try {
            dai::boundTreewidth(factor_graph, &dai::eliminationCost_MinFill, maxstates);
        } catch( dai::Exception &e ) {
            if( e.getCode() == dai::Exception::OUT_OF_MEMORY )
                std::cout << "error: cannot compute junction tree (need more than " << maxstates << " states)" << std::endl;
        }
#endif

        // junction tree
        dai::PropertySet opts;
        //opts = dai::PropertySet()("updates", std::string("HUGIN"))("verbose", (size_t)0);
        //inference_algorithm_ = new dai::JTree(factor_graph, opts);

        opts = dai::PropertySet()("updates", std::string("SEQRND"))("logdomain", false)("tol", 1e-3)("maxiter", (size_t)20)("maxtime", double(1))("damping", double(.2))("verbose", (size_t)0);
        inference_algorithm_ = new dai::BP(factor_graph, opts);

        //opts = dai::PropertySet()("updates", std::string("SEQRND"))("clamp", std::string("CLAMP_VAR"))("choose", std::string("CHOOSE_RANDOM"))("min_max_adj", double(10))("bbp_props", std::string(""))("bbp_cfn", std::string(""))("recursion", std::string("REC_FIXED"))("tol", 1e-3)("rec_tol", 1e-3)("maxiter", (size_t)100)("verbose", (size_t)0);
        //inference_algorithm_ = new dai::CBP(factor_graph, opts);

        //opts = dai::PropertySet()("updates", std::string("SEQRND"))("cavity", std::string("FULL"))("logdomain", false)("tol", 1e-3)("maxiter", (size_t)100)("maxtime", double(1))("damping", double(.2))("verbose", (size_t)0);
        //inference_algorithm_ = new dai::LC(factor_graph, opts);

        //opts = dai::PropertySet()("updates", std::string("LINEAR"))("inits", std::string("RESPPROP"))("logdomain", false)("tol", 1e-3)("maxiter", (size_t)100)("maxtime", double(1))("damping", double(.2))("verbose", (size_t)10);
        //inference_algorithm_ = new dai::MR(factor_graph, opts);

        //opts = dai::PropertySet()("doubleloop", true)("clusters", std::string("MIN"))("init", std::string("UNIFORM"))("tol", 1e-3)("maxiter", (size_t)100)("maxtime", double(1));
        //inference_algorithm_ = new dai::HAK(factor_graph, opts);

        std::cout << "Properties: " << inference_algorithm_->printProperties() << std::endl;
        inference_algorithm_->init();
    }

    void destroy_inference_algorithm() {
        delete inference_algorithm_;
    }

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

    void print_factor_edbp(std::ostream &os, const dai::Factor &factor) const {
        os << factor.nrStates() << std::endl;
        for( int j = 0; j < int(factor.nrStates()); ++j )
            os << " " << factor[j];
        os << std::endl;
    }

    void initial_sampling_in_place() {
        assert(base_->nlabels_ == 2);

        // set initial history and reset factors for locations
        loc_history_.push_back(base_->initial_loc_);
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / (1 << factor.vars().size());
            for( int i = 0; i < (1 << factor.vars().size()); ++i )
                factor.set(i, p);
        }
        calculate_marginals();
        //TEST(std::cout, "INITIAL");
    }

    void TEST(std::ostream &os, const std::string &str, bool do_endl = true) const {
        os << str << ": p=" << this << ", t=" << std::flush;
        assert(inference_algorithm_->fg().nrFactors() == factors_.size());
        for( int i = 0; i < int(factors_.size()); ++i ) {
            os << " " << i << std::flush;
            assert(inference_algorithm_->fg().factor(i) == factors_[i]);
        }
        if( do_endl ) os << std::endl;
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

    int update_factors(int last_action, int obs) {
        int current_loc = loc_history_.back();
#ifdef DEBUG
        //std::cout << "update_factors: loc=" << current_loc << ", coord=" << coord_t(current_loc) << ", obs=" << obs << std::endl;
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        print_factor(std::cout, current_loc, factors_[current_loc], "factors_");
#endif
        assert(current_loc < int(factors_.size()));
        dai::Factor &factor = factors_[current_loc];
        float total_mass = 0.0;
        for( int j = 0; j < int(factor.nrStates()); ++j ) {
            //std::cout << "CASE j=" << std::setw(3) << j << std::endl;
            int slabels = get_slabels(current_loc, j);
            factor.set(j, factor[j] * base_->probability_obs_special(obs, current_loc, slabels, last_action));
            total_mass += factor[j];
        }
        assert(total_mass > 0);
        factor /= total_mass;
        need_to_recalculate_marginals_ = true;
#ifdef DEBUG
        std::cout << "factor after  update: loc=" << current_loc << ", obs=" << obs << std::endl;
        print_factor(std::cout, current_loc, factors_[current_loc], "factors_");
#endif
        return current_loc;
    }

    void calculate_marginals(int factor_index = -1, bool print_marginals = false) const {
        if( need_to_recalculate_marginals_ ) {
            apply_inference_libdai(factor_index);
            extract_marginals_from_inference_libdai(print_marginals);
            //if( print_marginals ) apply_inference_edbp();
            need_to_recalculate_marginals_ = false;
        }
    }

    void update_marginal(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            //dai::Factor marginal = marginals_[loc].marginal(factors_[loc].vars());
            dai::Factor marginal = marginals_[loc].marginal(dai::VarSet(dai::Var(loc, 2))); // CHECK
            assert(base_->nlabels_ == int(marginal.nrStates()));
            for( int label = 0; label < base_->nlabels_; ++label )
                marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * marginal[label]);
        }
        int current_loc = loc_history_.back();
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
    }

    int value_for(int /*var*/) const { return -1; }

    virtual void sample_from_pi(rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const = 0;
    virtual float importance_weight(const rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const = 0;

    void apply_inference_libdai(int factor_index) const {
        assert(inference_algorithm_ != 0);
        if( factor_index == -1 ) {
            for( int i = 0; i < int(factors_.size()); ++i )
                inference_algorithm_->fg().setFactor(i, factors_[i]);
            inference_algorithm_->init();
        } else {
            //std::cout << "p=" << this << ": index=" << factor_index << ", ";
            inference_algorithm_->fg().setFactor(factor_index, factors_[factor_index]);
            assert(inference_algorithm_->fg().factor(factor_index) == factors_[factor_index]);
            //TEST(std::cout, "APPLY", false);
            inference_algorithm_->init(factors_[factor_index].vars());
            //std::cout << std::endl;
        }
        inference_algorithm_->run();
    }

    void extract_marginals_from_inference_libdai(bool print_marginals = false) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            marginals_[loc] = inference_algorithm_->belief(factors_[loc].vars());
            if( print_marginals )
                print_factor(std::cout, loc, marginals_[loc], "marginals_");
        }
    }

    void apply_inference_edbp() const {
        // create model in file dummy.uai
        std::ofstream ofs("dummy.uai");
        ofs << "MARKOV"
            << std::endl
            << base_->nloc_
            << std::endl;
        for( int loc = 0; loc < base_->nloc_; ++loc )
            ofs << 2 << (loc < base_->nloc_ - 1 ? " " : "");
        ofs << std::endl << base_->nloc_ << std::endl;
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            const dai::Factor &factor = inference_algorithm_->fg().factor(loc);
            ofs << factor.vars().size();
            for( dai::VarSet::const_reverse_iterator it = factor.vars().rbegin(); it != factor.vars().rend(); ++it ) {
                ofs << " " << it->label();
            }
            ofs << std::endl;
        }
        ofs << std::endl;
        for( int loc = 0; loc < int(factors_.size()); ++loc ) {
            print_factor_edbp(ofs, inference_algorithm_->fg().factor(loc));
            ofs << std::endl;
        }
        ofs.close();

        // call edbp solver
        //system("~/software/edbp/solver dummy.uai dummy.evid 0 MAR 2>/dev/null");
    }

    void extract_marginals_from_inference_edbp(bool print_marginals = false) const {
    }
};

// Particle for the motion model RBPF filter (slam2)
struct motion_model_rbpf_slam2_particle_t : public rbpf_slam2_particle_t {
    virtual void sample_from_pi(rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        assert(np == p);
        int next_loc = base_->sample_loc(p.loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        int factor_index = np.update_factors(last_action, obs);
        np.calculate_marginals(factor_index, false);
    }

    virtual float importance_weight(const rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        assert(!p.need_to_recalculate_marginals_);
        int np_current_loc = np.loc_history_.back();
        float weight = 0;
        const dai::Factor &p_marginal = p.marginals_[np_current_loc];
        for( int slabels = 0; slabels < int(p_marginal.nrStates()); ++slabels )
            weight += p_marginal[slabels] * base_->probability_obs_special(obs, np_current_loc, slabels, last_action);
        return weight;
    }

    motion_model_rbpf_slam2_particle_t* initial_sampling() {
        motion_model_rbpf_slam2_particle_t *p = new motion_model_rbpf_slam2_particle_t;
        p->initial_sampling_in_place();
        return p;
    }
};

// Particle for the optimal RBPF filter (verified: 09/12/2015)
struct optimal_rbpf_slam2_particle_t : public rbpf_slam2_particle_t {
    void calculate_cdf(const rbpf_slam2_particle_t &p, int last_action, int obs, std::vector<float> &cdf) const {
        assert(!p.need_to_recalculate_marginals_);

        cdf.clear();
        cdf.reserve(base_->nloc_);

        int current_loc = p.loc_history_.back();
        float previous = 0;

        const dai::Factor &p_marginal = p.marginals_[current_loc];
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            float prob = 0;
            for( int slabels = 0; slabels < int(p_marginal.nrStates()); ++slabels )
                prob += p_marginal[slabels] * base_->probability_obs_special(obs, new_loc, slabels, last_action);
            cdf.push_back(previous + base_->probability_tr_loc(last_action, current_loc, new_loc) * prob);
            previous = cdf.back();
        }

        // normalize
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            cdf[new_loc] /= cdf.back();
        }
        assert(cdf.back() == 1.0);
    }

    virtual void sample_from_pi(rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        assert(np == p);
        //p.TEST(std::cout, "SAMPLE1");
        //np.TEST(std::cout, "SAMPLE2");
        std::vector<float> cdf;
        calculate_cdf(p, last_action, obs, cdf);
        int next_loc = Utils::sample_from_distribution(base_->nloc_, &cdf[0]);
        np.loc_history_.push_back(next_loc);
        int factor_index = np.update_factors(last_action, obs);
        np.calculate_marginals(factor_index, false);
        //np.TEST(std::cout, "SAMPLE3");
    }

    virtual float importance_weight(const rbpf_slam2_particle_t &/*np*/, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        assert(!p.need_to_recalculate_marginals_);
        int current_loc = p.loc_history_.back();
        float weight = 0;
        const dai::Factor &p_marginal = p.marginals_[current_loc];
        for( int new_loc = 0; new_loc < base_->nloc_; ++new_loc ) {
            float prob = 0;
            for( int slabels = 0; slabels < int(p_marginal.nrStates()); ++slabels )
                prob += p_marginal[slabels] * base_->probability_obs_special(obs, new_loc, slabels, last_action);
            weight += base_->probability_tr_loc(last_action, current_loc, new_loc) * prob;
        }
        return weight;
    }

    optimal_rbpf_slam2_particle_t* initial_sampling() {
        optimal_rbpf_slam2_particle_t *p = new optimal_rbpf_slam2_particle_t;
        p->initial_sampling_in_place();
        return p;
    }
};

#undef DEBUG

#endif

