/*
 *  Copyright (C) 2012 Universidad Simon Bolivar
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


#ifndef LOOPY_BELIEF_PROPAGATION_H
#define LOOPY_BELIEF_PROPAGATION_H

#include "factor_graph.h"

#include <iostream>
#include <cassert>

#define DEBUG

namespace BP {

template<typename T> class loopy_belief_propagation_t {
  public:
    typedef factor_graph_t::edge_list_t edge_list_t;
    typedef factor_graph_t::edge_t edge_t;

  private:
    std::vector<int> factor_worklist_;
    std::vector<bool> factors_in_worklist_;
    std::vector<int> variable_worklist_;
    std::vector<bool> variables_in_worklist_;

  protected:
    int nvars_;
    int nfactors_;
    variable_t<T> **variables_;
    abstract_factor_t<T> **factors_;
    const factor_graph_t &graph_;
    bool simulate_arc_consistency_;

    int workspace_size_;
    mutable T *workspace_;

  public:
    loopy_belief_propagation_t(const factor_graph_t &graph, bool simulate_arc_consistency = false)
      : nvars_(graph.nvars()),
        nfactors_(graph.nfactors()),
        variables_(new variable_t<T>*[nvars_]),
        factors_(new abstract_factor_t<T>*[nfactors_]),
        graph_(graph),
        simulate_arc_consistency_(simulate_arc_consistency),
        workspace_size_(0),
        workspace_(0) {
        memset(variables_, 0, nvars_ * sizeof(variable_t<T>*));
        memset(factors_, 0, nfactors_ * sizeof(abstract_factor_t<T>*));
        factors_in_worklist_ = std::vector<bool>(nfactors_, false);
        variables_in_worklist_ = std::vector<bool>(nvars_, false);
    }
    explicit loopy_belief_propagation_t(const loopy_belief_propagation_t &lbp)
      : nvars_(lbp.nvars_),
        nfactors_(lbp.nfactors_),
        variables_(new variable_t<T>*[nvars_]),
        factors_(new abstract_factor_t<T>*[nfactors_]),
        graph_(lbp.graph_),
        simulate_arc_consistency_(lbp.simulate_arc_consistency_),
        workspace_size_(lbp.workspace_size_),
        workspace_(new T[workspace_size_]) {
        memcpy(variables_, lbp.variables_, nvars_ * sizeof(variable_t<T>*));
        memcpy(factors_, lbp.factors_, nfactors_ * sizeof(abstract_factor_t<T>*));
        factors_in_worklist_ = std::vector<bool>(nfactors_, false);
        variables_in_worklist_ = std::vector<bool>(nvars_, false);
    }
    loopy_belief_propagation_t(loopy_belief_propagation_t &&lbp)
      : nvars_(lbp.nvars_),
        nfactors_(lbp.nfactors_),
        variables_(lbp.variables_),
        factors_(lbp.factors_),
        graph_(std::move(lbp.graph_)),
        simulate_arc_consistency_(lbp.simulate_arc_consistency_),
        workspace_size_(lbp.workspace_size_),
        workspace_(lbp.workspace_) {
        lbp.variables_ = 0;
        lbp.factors_ = 0;
        lbp.workspace_ = 0;
        factors_in_worklist_ = std::vector<bool>(nfactors_, false);
        variables_in_worklist_ = std::vector<bool>(nvars_, false);
    }
    ~loopy_belief_propagation_t() {
        delete[] variables_;
        delete[] factors_;
        delete[] workspace_;
    }

    // set variables and factors
    variable_t<T>* variable(int vid) { return variables_[vid]; }
    void set_variable(int vid, variable_t<T> *variable) {
        assert(vid < nvars_);
        variables_[vid] = variable;
    }

    abstract_factor_t<T>* factor(int fid) { return factors_[fid]; }
    void set_factor(int fid, abstract_factor_t<T> *factor) {
        assert(fid < nfactors_);
        factors_[fid] = factor;
    }

    // access to marginals
    const T* marginal(int vid) const {
        assert((vid >= 0) && (vid < nvars_));
        return variables_[vid] == 0 ? 0 : variables_[vid]->marginal();
    }

    void compute_marginal(int vid) {
        variable_t<T> *variable = variables_[vid];
        if( variable != 0 ) variable->compute_marginal();
    }
    void compute_marginals() {
        for( int vid = 0; vid < nvars_; ++vid )
            compute_marginal(vid);
    }

    // initialize variables and factors
    void initialize() {
        for( int vid = 0; vid < nvars_; ++vid ) {
            variable_t<T> *variable = variables_[vid];
            if( variable != 0 ) variable->initialize(nfactors_);
        }

        workspace_size_ = 0;
        for( int fid = 0; fid < nfactors_; ++fid ) {
            abstract_factor_t<T> *factor = factors_[fid];
            if( factor != 0 ) {
                factor->initialize(nvars_);
                workspace_size_ = std::max(workspace_size_, factor->factorsz_);
            }
        }
        workspace_ = new T[workspace_size_];

#ifdef DEBUG
        std::cout << "lbp: workspace-size=" << workspace_size_ << std::endl;
#endif
    }

    // Messages: variables to factors
    const T* variable_to_factor_message(int vid, int fid) const {
        return variables_[vid] != 0 ? variables_[vid]->message_to_factor_by_fid(fid) : 0;
    }

    // Messages: factors to variables
    const T* factor_to_variable_message(int fid, int vid) const {
        return factors_[fid] != 0 ? factors_[fid]->message_to_variable_by_vid(vid) : 0;
    }

    // Worklist
    void add_factor_to_worklist(int fid) {
        if( !factors_in_worklist_[fid] ) {
            factors_in_worklist_[fid] = true;
            factor_worklist_.push_back(fid);
        }
    }

    void add_variable_to_worklist(int vid) {
        if( !variables_in_worklist_[vid] ) {
            variables_in_worklist_[vid] = true;
            variable_worklist_.push_back(vid);
        }
    }

    void calculate_variables_to_revise(int fid) {
        assert(factors_[fid] != 0);
        const abstract_factor_t<T> &factor = *factors_[fid];

        // revise next message to check if some entry change from zero to non-zero (or vice versa)
        for( int vix = 0; vix < factor.nvars_; ++vix ) {
            int vid = factor.variables_[vix]->id_;
            const T *stored_msg = factor.message_to_variable_by_vix(vix);
            const T *next_msg = &factor.next_messages_[factor.offsets_[vix]];
            for( int i = 0; i < factor.variables_[vix]->domsz_; ++i ) {
                if( ((next_msg[i] == 0) && (stored_msg[i] != 0)) ||
                    ((next_msg[i] != 0) && (stored_msg[i] == 0)) ) {
                    if( !variables_in_worklist_[vid] ) {
                        variables_in_worklist_[vid] = true;
                        variable_worklist_.push_back(vid);
#ifdef DEBUG
                        //std::cout << "mp: enqueueing variable vid=" << vid << std::endl;
#endif
                        break;
                    }
                }
            }
        }
    }
    void calculate_factors_to_revise(int vid) {
        assert(variables_[vid] != 0);
        const variable_t<T> &variable = *variables_[vid];

        // revise next message to check if some entry change from zero to non-zero (or vice versa)
        for( int fix = 0; fix < variable.nfactors_; ++fix ) {
            int fid = variable.factors_[fix]->id_;
            const T *stored_msg = variable.message_to_factor_by_fix(fix);
            const T *next_msg = &variable.next_messages_[fix * variable.domsz_];
            for( int i = 0; i < variable.domsz_; ++i ) {
                if( ((next_msg[i] == 0) && (stored_msg[i] != 0)) ||
                    ((next_msg[i] != 0) && (stored_msg[i] == 0)) ) {
                    if( !factors_in_worklist_[fid] ) {
                        factors_in_worklist_[fid] = true;
                        factor_worklist_.push_back(fid);
#ifdef DEBUG
                        //std::cout << "mp: enqueueing factor fid=" << fid << std::endl;
#endif
                        break;
                    }
                }
            }
        }
    }

    void set_default_messages() {
        for( int vid = 0; vid < nvars_; ++vid ) {
            if( variables_[vid] != 0 )
                variables_[vid]->fill_messages_by_default();
        }
        for( int fid = 0; fid < nfactors_; ++fid ) {
            if( factors_[fid] != 0 )
                factors_[fid]->fill_messages_by_default();
        }
    }

    // Message Passing implementation of Loopy Belief Propagation.
    //
    // We implement a schedule following the idea oc AC3 in which edges get
    // scheduled whenever one of its nodes' domain is changed (i.e., an element
    // is pruned/inserted into the domain).
    int mp(int max_num_iterations, std::vector<int> &revised_vars) {
        //std::cout << "lbp: mp: begin" << std::endl;
        // CHECK: need to find good scheduling

        // allocate space for revised vars to keep track of them
        std::vector<bool> inserted(nvars_, false);
        revised_vars.reserve(nvars_);
        revised_vars.clear();

        // revise arcs until max number of iterations reached or
        // worklist becomes empty
        int num_iterations = 0;
        while( (num_iterations < max_num_iterations) &&
               (!factor_worklist_.empty() || !variable_worklist_.empty()) ) {
            ++num_iterations;

            // fetch edge from worklist
            if( !variable_worklist_.empty() ) {
                int i = lrand48() % variable_worklist_.size();
                int vid = variable_worklist_[i];
                variable_worklist_[i] = variable_worklist_.back();
                variable_worklist_.pop_back();
                assert(variables_in_worklist_[vid]);
                variables_in_worklist_[vid] = false;
                //std::cout << "dequeued vid=" << vid << ", qsz=" << variable_worklist_.size() << std::endl;

                // Compute outboud messages for factor fid. If some entry
                // for message to variable vid becomes zero or non-zero,
                // then queue edge fid --> vid in factor queue.
                variables_[vid]->calculate_outbound_messages();
                //variables_[vid]->print_messages(std::cout);
                calculate_factors_to_revise(vid);
                variables_[vid]->update_outbound_messages();
            } else {
                int i = lrand48() % factor_worklist_.size();
                int fid = factor_worklist_[i];
                factor_worklist_[i] = factor_worklist_.back();
                factor_worklist_.pop_back();
                assert(factors_in_worklist_[fid]);
                factors_in_worklist_[fid] = false;
                //std::cout << "dequeued fid=" << fid
                //          << ", f.nvars=" << factors_[fid]->nvars_
                //          << ", qsz=" << factor_worklist_.size()
                //          << std::endl;

                // Compute outboud messages for variable vid. If some entry
                // for message to factor fid becomes zero or non-zero, then
                // queue edge vid --> fid in variable queue.
                factors_[fid]->calculate_outbound_messages(workspace_);
                //factors_[fid]->print_messages(std::cout);
                calculate_variables_to_revise(fid);
                factors_[fid]->update_outbound_messages();
            }
        }

        factors_in_worklist_ = std::vector<bool>(nfactors_, false);
        variables_in_worklist_ = std::vector<bool>(nvars_, false);
        factor_worklist_.clear();
        variable_worklist_.clear();
        //std::cout << "lbp: mp: end" << std::endl;
        return num_iterations;
    }

    void fwmp(int num_iterations) {
        // set initial messages
        set_default_messages();

        std::cout << "fwmp: iterations=";
        for( int iter = 0; iter < num_iterations; ++iter ) {
            std::cout << iter << " " << std::flush;
            for( int fid = 0; fid < nfactors_; ++fid ) {
                if( factors_[fid] != 0 ) {
                    factors_[fid]->calculate_outbound_messages(workspace_);
                    //factors_[fid]->update_outbound_messages();
                }
            }
            for( int fid = 0; fid < nfactors_; ++fid ) {
                if( factors_[fid] != 0 ) {
                    //factors_[fid]->calculate_outbound_messages(workspace_);
                    factors_[fid]->update_outbound_messages();
                }
            }
            for( int vid = 0; vid < nvars_; ++vid ) {
                if( variables_[vid] != 0 ) {
                    variables_[vid]->calculate_outbound_messages();
                    //variables_[vid]->update_outbound_messages();
                }
            }
             for( int vid = 0; vid < nvars_; ++vid ) {
                if( variables_[vid] != 0 ) {
                    //variables_[vid]->calculate_outbound_messages();
                    variables_[vid]->update_outbound_messages();
                }
            }
        }
        std::cout << std::endl;
    }

};

}; // end of namespace BP

#undef DEBUG

#endif

