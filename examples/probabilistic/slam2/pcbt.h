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

#ifndef PCBT_H
#define PCBT_H

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


// (Standard) Probabilistic Causal Belief Tracking
template <typename BASE> struct pcbt_t : public tracking_t<BASE> {
    using tracking_t<BASE>::base_;

    int memory_;
    int prefix_mask_;
    std::vector<dai::Var> variables_;
    std::vector<dai::Factor> factors_;

    bool project_join_;
    bool marginals_calculated_;

    std::string algorithm_;
    dai::PropertySet opts_;

    mutable dai::FactorGraph fg_;
    mutable dai::InfAlg *inference_algorithm_;

    pcbt_t(const std::string &name, const BASE &base, int memory, bool project_join = false)
      : tracking_t<BASE>(name, base), memory_(memory), project_join_(project_join)  {
        prefix_mask_ = 1;
        for( int i = 0; i < memory_; ++i ) prefix_mask_ *= base_.nloc_;
        //std::cerr << "pcbt: memory=" << memory_ << ", prefix=" << prefix_mask_ << std::endl;
        inference_algorithm_ = 0;
        marginals_calculated_ = false;
        create_variables_and_factors();
    }
    virtual ~pcbt_t() { }

    void create_variables_and_factors() {
        variables_ = std::vector<dai::Var>(base_.nvars_);
        for( int i = 0; i < base_.nloc_; ++i )
            variables_[i] = dai::Var(i, base_.nlabels_);
        variables_[base_.nloc_] = dai::Var(base_.nloc_, prefix_mask_ * base_.nloc_);
        factors_ = std::vector<dai::Factor>(base_.nloc_);
    }

    void reset_factors(size_t initial_loc) {
        int init_loc_hist = initial_loc;
        for( int i = 0; i < memory_; ++i )
            init_loc_hist = init_loc_hist * base_.nloc_ + initial_loc;

        for( int i = 0; i < base_.nloc_; ++i ) {
            dai::VarSet factor_vars(variables_[i], variables_[base_.nloc_]);
            factors_[i] = dai::Factor(factor_vars, 0.0);

            // set valuations for initial loc (memory loc variables are also set to initial loc)
            for( int label = 0; label < base_.nlabels_; ++label )
                factors_[i].set(init_loc_hist * base_.nlabels_ + label, 1.0);
        }
    }

    void progress_factor_with_action(int /*findex*/, dai::Factor &factor, int last_action) const {
        //std::cerr << "    PROGRESS[action=" << last_action << "]:"
        //          << " old factor[" << findex << "]: " << factor << std::endl;
        if( last_action != -1 ) {
            dai::Factor new_factor(factor.vars(), 0.0);
            for( size_t j = 0; j < factor.nrStates(); ++j ) {
                int label = j % base_.nlabels_;
                int loc_hist = j / base_.nlabels_;
                int loc = loc_hist % base_.nloc_;
                int prefix_loc_hist = loc_hist % prefix_mask_;
                float weight = factor[j];
                for( int new_loc = 0; new_loc < base_.nloc_; ++new_loc ) {
                    int new_loc_hist = prefix_loc_hist * base_.nloc_ + new_loc;
                    float new_weight = weight * base_.probability_tr_loc(last_action, loc, new_loc);
                    int index = new_loc_hist * base_.nlabels_ + label;
                    assert(index < int(new_factor.nrStates()));
                    if( new_weight != 0 ) new_factor.set(index, new_factor[index] + new_weight);
                }
            }
            factor = new_factor;
        }
        //std::cerr << "    PROGRESS[action=" << last_action << "]:"
        //          << " new factor[" << findex << "]: " << factor << std::endl;
    }

    void filter_factor_with_obs(int findex, dai::Factor &factor, int last_action, int obs) const {
        //std::cerr << "    FILTER[obs=" << obs << "]:"
        //          << " old factor[" << findex << "]: " << factor << std::endl;
        if( obs != -1 ) {
            for( size_t j = 0; j < factor.nrStates(); ++j ) {
                float weight = factor[j];
                int label = j % base_.nlabels_;
                int loc_hist = j / base_.nlabels_;
                int loc = loc_hist % base_.nloc_;
                if( findex == loc ) { // findex == "most recent loc in loc_hist"
                    factor.set(j, weight * base_.probability_obs(obs, loc, label, last_action));
                } else {
                    factor.set(j, weight / float(base_.nlabels_));
                }
            }
        }
        //std::cerr << "    FILTER[obs=" << obs << "]:"
        //          << " new factor[" << findex << "]: " << factor << std::endl;
    }

    void marginalize_beams() {
        for( int i = 0; i < int(factors_.size()); ++i ) {
            std::cerr << "MARGINALIZE: old factor[" << i << "]: " << factors_[i] << std::endl;
            dai::Factor &factor = factors_[i];
            const dai::Factor &new_factor = inference_algorithm_->belief(factor.vars());
            factor = new_factor;
            std::cerr << "MARGINALIZE: new factor[" << i << "]: " << factors_[i] << std::endl;
        }
    }

    void update_factor(int i, int last_action, int obs) {
        //std::cerr << "UPDATE[last_action=" << last_action << ", obs=" << obs << "]:"
        //          << " old factor[" << i << "]: " << factors_[i] << std::endl;
        progress_factor_with_action(i, factors_[i], last_action);
        filter_factor_with_obs(i, factors_[i], last_action, obs);
        //std::cerr << "UPDATE[last_action=" << last_action << ", obs=" << obs << "]:"
        //          << " new factor[" << i << "]: " << factors_[i] << std::endl;
    }

    void set_algorithm_and_options(const std::string &algorithm, const dai::PropertySet &opts) {
        algorithm_ = algorithm;
        opts_ = opts;
    }

    virtual void initialize() {
        reset_factors(base_.initial_loc_);
    }

    virtual void update(int last_action, int obs) {
        assert((obs >= 0) && (obs < base_.nlabels_));
        for( int i = 0; i < int(factors_.size()); ++i )
            update_factor(i, last_action, obs);
        marginals_calculated_ = false;

        if( project_join_ ) {
            calculate_marginals_internal();
            marginalize_beams();
        }
    }

    void calculate_marginals_internal() {
        fg_ = dai::FactorGraph(factors_);
        delete inference_algorithm_;
        if( algorithm_ == "JT" )
            inference_algorithm_ = new dai::JTree(fg_, opts_);
        else if( algorithm_ == "BP" )
            inference_algorithm_ = new dai::BP(fg_, opts_);
        else if( algorithm_ == "HAK" )
            inference_algorithm_ = new dai::HAK(fg_, opts_);
        inference_algorithm_->init();
        inference_algorithm_->run();
        marginals_calculated_ = true;
    }
    virtual void calculate_marginals() {
        if( !marginals_calculated_ ) calculate_marginals_internal();
        if( project_join_ ) marginalize_beams();
    }

    virtual void get_marginal(int var, dai::Factor &marginal) const {
        if( !project_join_ ) {
            const dai::Factor &factor = inference_algorithm_->belief(fg_.var(var));
            if( var < base_.nloc_ ) {
                marginal = factor;
            } else {
                marginal = dai::Factor(dai::VarSet(dai::Var(base_.nloc_, base_.nloc_)), 0.0);
                for( size_t i = 0; i < factor.nrStates(); ++i )
                    marginal.set(i % base_.nloc_, marginal[i % base_.nloc_] + factor[i]);
            }
        } else {
            if( var < base_.nloc_ ) {
                marginal = factors_[var];
            } else {
                const dai::Factor &factor = factors_[0];
                marginal = dai::Factor(dai::VarSet(dai::Var(base_.nloc_, base_.nloc_)), 0.0);
                for( size_t i = 0; i < factor.nrStates(); ++i ) {
                    int loc_hist = i / base_.nlabels_;
                    marginal.set(loc_hist % base_.nloc_, marginal[loc_hist % base_.nloc_] + factor[i]);
                }
            }
        }
    }

    virtual float* get_marginal(int var, float *ptr) const {
        assert(0);
    }
};

#endif

