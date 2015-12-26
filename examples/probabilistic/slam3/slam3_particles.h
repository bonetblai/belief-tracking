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

#ifndef SLAM3_PARTICLES_H
#define SLAM3_PARTICLES_H

#include <cassert>
#include <cstdlib>
#include <string>
#include <iostream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>

#include <dai/alldai.h>

#include "cellmap.h"
#include "slam_particles.h"
#include "inference.h"
#include "utils.h"

#include "var_beam.h"
#include "arc_consistency.h"

//#define DEBUG

class varset_beam_t : public var_beam_t {
  protected:
    const int loc_;
    const dai::Var var_;
    const dai::VarSet varset_;
    unsigned mask_;

    float alpha_;
    static float kappa_;

  public:
    varset_beam_t(int loc, const dai::Var &var, const dai::VarSet &varset)
      : var_beam_t(1, varset.nrStates()), loc_(loc), var_(var), varset_(varset), mask_(0), alpha_(-1) {
        mask_ = unsigned(-1);
        for( dai::State state(varset_); state.valid(); state++ ) {
            if( state(var_) == 1 )
                mask_ = mask_ & dai::calcLinearState(varset_, state);
        }
#ifdef DEBUG
        std::cout << "varset_beam_t: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    varset_beam_t(const varset_beam_t &beam)
      : var_beam_t(beam),
        loc_(beam.loc_),
        var_(beam.var_),
        varset_(beam.varset_),
        mask_(beam.mask_),
        alpha_(beam.alpha_) {
#ifdef DEBUG
        std::cout << "varset_beam_t: copy-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    varset_beam_t(varset_beam_t &&beam)
      : var_beam_t(std::move(beam)),
        loc_(beam.loc_),
        var_(beam.var_),
        varset_(std::move(beam.varset_)),
        mask_(beam.mask_),
        alpha_(beam.alpha_) {
#ifdef DEBUG
        std::cout << "varset_beam_t: move-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    ~varset_beam_t() { }

    static float kappa() { return kappa_; }
    static void set_kappa(float kappa) { kappa_ = kappa; }

    bool operator==(const varset_beam_t &beam) const {
        return (loc_ == beam.loc_) && (*static_cast<const var_beam_t*>(this) == *static_cast<const var_beam_t*>(&beam));
    }

    int loc() const { return loc_; }
    const dai::VarSet& varset() const { return varset_; }

    float probability(int valuation) const {
        // need to consider case where valuation is inside/outside csp. In the former case,
        // probability is alpha / number-valuations-inside. In the latter case, it is 
        // alpha * kappa / number-valuations-outside.
        float p = contains(valuation) ? 1.0 : kappa_;
        //std::cout << "probability: p=" << p << ", mask=" << mask_ << ", alpha=" << alpha_ << std::endl;
        return alpha_ * p;
    }
    float marginal(int label) const {
        float p = 0;
        for( unsigned valuation = 0; valuation < unsigned(varset_.nrStates()); ++valuation ) {
            // if valuation is compatible with lab, p += probability(valuation)
            if( ((label == 1) && ((valuation & mask_) != 0)) || ((label == 0) && ((valuation & mask_) == 0)) )
                p += probability(valuation);
        }
        return p;
    }

#if 0
    using var_beam_t::contains;
    bool contains(const dai::VarSet &vars, const std::map<dai::Var, size_t> &state) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            std::map<dai::Var, size_t> s = dai::calcState(varset_, *it);
            bool match = true;
            for( dai::VarSet::const_iterator jt = vars.begin(); match && (jt != vars.end()); ++jt ) {
                std::map<dai::Var, size_t>::const_iterator xt = state.find(*jt);
                std::map<dai::Var, size_t>::const_iterator yt = s.find(*jt);
                assert(xt != state.end());
                assert(yt != s.end());
                match = xt->second == yt->second;
            }
            if( match ) return true;
        }
        return false;
    }

    int label(const dai::Var &var, int valuation) const {
        std::map<dai::Var, size_t> state;
        state = dai::calcState(varset_, valuation);
        return state[var];
    }
#endif

    void set_alpha() {
        alpha_ = 1.0 / float(size() + (varset_.nrStates() - size()) * kappa_);
    }
    void set_initial_configuration() {
        var_beam_t::set_initial_configuration();
        set_alpha();
    }
    void erase_ordered_indices(const std::vector<int> &ordered_indices) {
        var_beam_t::erase_ordered_indices(ordered_indices);
    }
};

class arc_consistency_t : public CSP::arc_consistency_t<varset_beam_t> {
    static CSP::constraint_digraph_t cg_;
    static std::vector<const char*> compatible_values_;

    mutable std::map<dai::Var, size_t> state_x_;
    mutable char bitmask_[64];

    static void construct_constraint_graph(int nrows, int ncols) {
        int num_locs = nrows * ncols;
        cg_.create_empty_graph(num_locs);
        for( int loc = 0; loc < num_locs; ++loc ) {
            cg_.reserve_edge_list(loc, 8);
            int r = loc / ncols, c = loc % ncols;
            for( int dr = -2; dr < 3; ++dr ) {
                if( (r + dr < 0) || (r + dr >= nrows) ) continue;
                for( int dc = -2; dc < 3; ++dc ) {
                    if( (c + dc < 0) || (c + dc >= ncols) ) continue;
                    if( (dr == 0) && (dc == 0) ) continue;
                    int nloc = (r + dr) * ncols + (c + dc);
                    cg_.add_edge(loc, nloc);
                }
            }
        }
        std::cout << "# cg: #vars=" << cg_.nvars() << ", #edges=" << cg_.nedges() << std::endl;
    }

    static void compute_compatible_values(int nrows, int ncols) {
        int num_locs = nrows * ncols;

        // variables for each location
        std::vector<dai::Var> variables(num_locs);
        for( int loc = 0; loc < num_locs; ++loc )
            variables[loc] = dai::Var(loc, 2);

        // variable sets for each location
        std::vector<dai::VarSet> varsets(num_locs);
        for( int loc = 0; loc < num_locs; ++loc ) {
            int row = loc / ncols, col = loc % ncols;
            std::vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= nrows) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= ncols) ) continue;
                    vars.push_back(variables[nr * ncols + nc]);
                }
            }
            std::sort(vars.begin(), vars.end());
            varsets[loc] = dai::VarSet(vars.begin(), vars.end());
#ifdef DEBUG
            std::cout << "# compute_compatible_values: loc=" << loc << ", vars=" << varsets[loc] << std::endl;
#endif
        }

        // for each location (var_y) and value for it, for each adjacent loc in constraint graph,
        // determine compatible values
        compatible_values_ = std::vector<const char*>(num_locs * num_locs * 512, 0);
        for( int var_y = 0; var_y < num_locs; ++var_y ) {
            int row = var_y / ncols, col = var_y % ncols;
            for( int val_y = 0; val_y < int(varsets[var_y].nrStates()); ++val_y ) {
                std::map<dai::Var, size_t> state_y = dai::calcState(varsets[var_y], val_y);
                for( int dr = -2; dr < 3; ++dr ) {
                    int nr = row + dr;
                    if( (nr < 0) || (nr >= nrows) ) continue;
                    for( int dc = -2; dc < 3; ++dc ) {
                        int nc = col + dc;
                        if( (nc < 0) || (nc >= ncols) ) continue;
                        int var_x = nr * ncols + nc;
                        char *values = new char[64];
                        bzero(values, 64);
                        int ncompatible = 0;
                        for( int val_x = 0; val_x < int(varsets[var_x].nrStates()); ++val_x ) {
                            bool compatible = true;
                            std::map<dai::Var, size_t> state_x = dai::calcState(varsets[var_x], val_x);
                            for( std::map<dai::Var, size_t>::const_iterator it = state_x.begin(); it != state_x.end(); ++it ) {
                                std::map<dai::Var, size_t>::const_iterator jt = state_y.find(it->first);
                                if( (jt != state_y.end()) && (jt->second != it->second) ) {
                                    compatible = false;
                                    break;
                                }
                            }
                            if( compatible ) {
                                assert(val_x < 512);
                                int index = val_x / 8, offset = val_x % 8;
                                values[index] |= 1 << offset;
                                ++ncompatible;
                            }
                        }
                        assert((ncompatible > 0) && (ncompatible < int(varsets[var_x].nrStates())));
                        assert(val_y * num_locs * num_locs + var_y * num_locs + var_x < int(compatible_values_.size()));
                        compatible_values_[val_y * num_locs * num_locs + var_y * num_locs + var_x] = values;
                    }
                }
            }
        }
    }

    const char * compatible_values(int var_x, int var_y, int val_y) const {
        assert(val_y * nvars() * nvars() + var_y * nvars() + var_x < int(compatible_values_.size()));
        assert(compatible_values_[val_y * nvars() * nvars() + var_y * nvars() + var_x] != 0);
        return compatible_values_[val_y * nvars() * nvars() + var_y * nvars() + var_x];
    }

    virtual void arc_reduce_preprocessing_1(int var_x, int val_x) const {
        state_x_ = dai::calcState(domain_[var_x]->varset(), val_x);
    }
    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const {
        std::map<dai::Var, size_t> state_y = dai::calcState(domain_[var_y]->varset(), val_y);
        for( std::map<dai::Var, size_t>::const_iterator it = state_x_.begin(); it != state_x_.end(); ++it ) {
            std::map<dai::Var, size_t>::const_iterator jt = state_y.find(it->first);
            if( (jt != state_y.end()) && (it->second != jt->second) ) return false;
        }
        return true;
    }

    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y) const { bzero(bitmask_, 64); }
    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y, int val_y) const {
        const char *bitmask = compatible_values(var_x, var_y, val_y);
        for( int i = 0; i < 64; ++i ) bitmask_[i] |= bitmask[i];
    }
    virtual bool arc_reduce_inverse_check(int val_x) const {
        int index = val_x / 8, offset = val_x % 8;
        return ((bitmask_[index] >> offset) & 0x1) == 1;
    }

  public:
    arc_consistency_t() : CSP::arc_consistency_t<varset_beam_t>(cg_) { }
    arc_consistency_t(const arc_consistency_t &ac) = delete;
    virtual ~arc_consistency_t() { }

    static void initialize_arc_consistency(int nrows, int ncols) {
        construct_constraint_graph(nrows, ncols);
        compute_compatible_values(nrows, ncols);
    }

    const arc_consistency_t& operator=(const arc_consistency_t &ac) {
        nvars_ = ac.nvars_;
        assert(domain_.size() == ac.domain_.size());
        for( int loc = 0; loc < int(ac.domain_.size()); ++loc )
            set_domain(loc, new varset_beam_t(*ac.domain(loc)));
        return *this;
    }
    bool operator==(const arc_consistency_t &ac) const {
        if( (nvars_ == ac.nvars_) && (domain_.size() == ac.domain_.size()) ) {
            for( int loc = 0; loc < int(domain_.size()); ++loc ) {
                if( !(*domain_[loc] == *ac.domain_[loc]) )
                    return false;
            }
            return true;
        } else {
            return false;
        }
    }

    bool ac3(bool inverse_check = false) {
        std::vector<int> revised_vars;
        bool something_removed = CSP::arc_consistency_t<varset_beam_t>::ac3(revised_vars, true, inverse_check);
        set_alpha(revised_vars);
        return something_removed;
    }

    void set_alpha(int loc) { domain_[loc]->set_alpha(); }
    void set_alpha(const std::vector<int> &revised_vars) {
        for( int i = 0; i < int(revised_vars.size()); ++i ) {
            int loc = revised_vars[i];
            set_alpha(loc);
        }
    }

    void print_factor(std::ostream &os, int loc) const {
        const varset_beam_t &beam = *domain_[loc];
        os << "loc=" << loc << ",sz=" << beam.size() << ",domain={";
        for( varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it )
            os << it.index() << ":" << *it << ",";
        os << "}";
    }
    void print(std::ostream &os) const {
        for( int loc = 0; loc < int(domain_.size()); ++loc ) {
            os << "[";
            print_factor(os, loc);
            os << "]" << std::endl;
        }
    }
};

// Abstract Particle for the 2nd Rao-Blackwellised filter
struct rbpf_slam3_particle_t : public base_particle_t {
    std::vector<int> loc_history_;

    const std::vector<int>& history() const {
        return loc_history_;
    }

    // arc consistency
    arc_consistency_t csp_;
    std::set<int> affected_beams_;

    // cache for conversion  from factor values into slabels (it is dynamically filled by get_slabels())
    static std::vector<std::vector<int> > slabels_;

    rbpf_slam3_particle_t() {
        assert(base_ != 0);
        assert(base_->nlabels_ == 2);

        // create binary variables for each cell in the grid
        int nloc = base_->nloc_;
        std::vector<dai::Var> variables(nloc);
        for( int loc = 0; loc < nloc; ++loc )
            variables[loc] = dai::Var(loc, 2);

        // create one factor for each location. The variables
        // in the factor are the variables for the location
        // surrounding the factor, including the variable
        // for the "center" location. Also set up the center
        // for each factor.
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
            csp_.set_domain(loc, new varset_beam_t(loc, variables[loc], varset));
        }
    }
    rbpf_slam3_particle_t(const rbpf_slam3_particle_t &p)
      : loc_history_(p.loc_history_) {
        csp_ = p.csp_;
    }
#if 0
    rbpf_slam3_particle_t(rbpf_slam3_particle_t &&p)
      : loc_history_(std::move(p.loc_history_)), csp_(std::move(p.csp_)) {
    }
#endif
    virtual ~rbpf_slam3_particle_t() {
        csp_.delete_domains_and_clear();
    }

    const rbpf_slam3_particle_t& operator=(const rbpf_slam3_particle_t &p) {
        loc_history_ = p.loc_history_;
        csp_ = p.csp_;
        return *this;
    }

    bool operator==(const rbpf_slam3_particle_t &p) const {
        return (loc_history_ == p.loc_history_) && (csp_ == p.csp_);
    }

    void reset_csp() {
        for( int loc = 0; loc < base_->nloc_; ++loc )
            csp_.domain(loc)->set_initial_configuration();
    }

    void initial_sampling_in_place(mpi_slam_t *mpi, int wid) {
        assert(base_->nlabels_ == 2);
        loc_history_.push_back(base_->initial_loc_);
        reset_csp();
        assert(csp_.is_consistent(0));
    }

    int get_slabels(int loc, const dai::VarSet &vars, int value) const {
        assert((loc >= 0) && (loc < base_->nloc_));
        assert((value >= 0) && (value < int(vars.nrStates())));

        // allocate cache if this is first call
        if( slabels_.empty() )
            slabels_ = std::vector<std::vector<int> >(base_->nloc_, std::vector<int>(512, -1));
        assert(loc < int(slabels_.size()));
        if( slabels_[loc].empty() )
            slabels_[loc] = std::vector<int>(vars.nrStates(), -1);
        assert(value < int(slabels_[loc].size()));

        // check whether there is a valid entry in cache
        const std::vector<int> &slabels_for_loc = slabels_[loc];
        if( slabels_for_loc[value] != -1 )
            return slabels_for_loc[value];

#ifdef DEBUG
        std::cout << "get_slabels(loc=" << loc << ":" << coord_t(loc) << ", value=" << value << "):" << std::endl;
#endif

        // this is the first time that we access (loc,value)
        // compute the correct value and cache it for later use
        int slabels = 0;
        std::map<dai::Var, size_t> state = dai::calcState(vars, value);
        for( dai::VarSet::const_iterator it = vars.begin(); it != vars.end(); ++it ) {
            const dai::Var &var = *it;
            size_t var_value = state[var];
            assert(var_value < 2); // because it is a binary variable
            if( var_value ) {
                int var_id = it->label();
                int var_off = base_->var_offset(loc, var_id);
                assert((var_off >= 0) && (var_off < 9));
#ifdef DEBUG
                std::cout << "    [var_id=" << var_id << ":" << coord_t(var_id)
                          << ", var_value=1, off=" << var_off << "]"
                          << std::endl;
#endif
                slabels += (1 << var_off);
            }
        }
#ifdef DEBUG
        std::cout << "    bits=|";
        print_bits(std::cout, slabels, 9);
        std::cout << "| (" << slabels << ")" << std::endl;
#endif

        // cache it and return
        slabels_[loc][value] = slabels;
        return slabels;
    }

    void calculate_pairs(varset_beam_t &beam, std::set<std::pair<int, int> > &pairs) const {
        pairs.clear();
        for( varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            std::map<dai::Var, size_t> state = dai::calcState(beam.varset(), *it);
            for( std::map<dai::Var, size_t>::const_iterator jt = state.begin(); jt != state.end(); ++jt )
                pairs.insert(std::make_pair(jt->first.label(), jt->second));
        }
    }

    int calculate_cost(int valuation, const varset_beam_t &beam, const std::set<std::pair<int, int> > &pairs) const {
        int cost = 0;
        std::map<dai::Var, size_t> state = dai::calcState(beam.varset(), valuation);
        for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it )
            cost += pairs.find(std::make_pair(it->first.label(), it->second)) == pairs.end() ? 1 : 0;
        return cost;
    }

    void common_variables(const varset_beam_t &beam1, const varset_beam_t &beam2, dai::VarSet &vars) const {
       vars = dai::VarSet();
       for( dai::VarSet::const_iterator it = beam1.varset().begin(); it != beam1.varset().end(); ++it ) {
           if( beam2.varset().contains(*it) )
               vars.insert(*it);
       }
#ifdef DEBUG
       std::cout << "common-vars: loc1=" << beam1.loc() << ", loc2=" << beam2.loc() << ", vars=" << vars << std::endl;
#endif
    }

    void recover_states_in_adjacent_beam(varset_beam_t &beam, const dai::VarSet &vars, const std::vector<std::pair<int, std::map<dai::Var, size_t> > > &states) {
        int i = 0;
        std::vector<std::map<dai::Var, size_t> > beam_states(beam.size());
        for( varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            beam_states[i++] = std::move(dai::calcState(beam.varset(), *it));
        }

        std::set<int> valuations_to_insert;
        for( int i = 0; i < int(states.size()); ++i ) {
            const std::map<dai::Var, size_t> &state = states[i].second;
            for( int j = 0; j < int(beam_states.size()); ++j ) {
                std::map<dai::Var, size_t> new_s = beam_states[j];
                for( dai::VarSet::const_iterator it = vars.begin(); it != vars.end(); ++it ) {
                    std::map<dai::Var, size_t>::const_iterator xt = state.find(*it);
                    std::map<dai::Var, size_t>::iterator yt = new_s.find(*it);
                    assert(xt != state.end());
                    assert(yt != new_s.end());
                    yt->second = xt->second;
                    assert(new_s[*it] == xt->second);
                }
                int valuation = dai::calcLinearState(beam.varset(), new_s);
                assert(valuation < beam.max_value());
                if( !beam.contains(valuation) ) {
                    valuations_to_insert.insert(valuation);
                }
            }
        }

        if( !valuations_to_insert.empty() ) {
            for( std::set<int>::const_iterator it = valuations_to_insert.begin(); it != valuations_to_insert.end(); ++it )
                beam.insert(*it);
            affected_beams_.insert(beam.loc());
        }
    }

    void recover_valuations(varset_beam_t &beam, const std::vector<std::pair<int, int> > &valuations) {
#if 1
        std::cout << "recover: loc=" << beam.loc() << ", #valuations=" << valuations.size() << std::flush;
#endif

        // compute states for valuations
        std::vector<std::pair<int, std::map<dai::Var, size_t> > > states(valuations.size());
        for( int i = 0; i < int(valuations.size()); ++i ) {
            int valuation = valuations[i].first;
            states[i].first = valuation;
            states[i].second = std::move(dai::calcState(beam.varset(), valuation));
        }

        // recover states in adjacent beams
#if 1
        std::cout << ", edges:";
#endif
        dai::VarSet vars;
        const CSP::constraint_digraph_t::edge_list_t &edge_list = csp_.digraph().edges_pointing_to(beam.loc());
        for( int i = 0; i < int(edge_list.size()); ++i ) {
            const CSP::constraint_digraph_t::edge_t &edge = edge_list[i];
#if 1
            std::cout << " (" << edge.first << "," << edge.second << ")" << std::flush;
#endif
            varset_beam_t &adj_beam = *csp_.domain(edge.first);
            common_variables(beam, adj_beam, vars);
            if( !vars.empty() )
#if 1
                std::cout << "adj-beam: loc=" << adj_beam.loc() << ", vars=" << vars << ", sz-before=" << adj_beam.size() << std::endl;
#endif
                recover_states_in_adjacent_beam(adj_beam, vars, states);
#if 1
                std::cout << "adj-beam: loc=" << adj_beam.loc() << ", sz-after=" << adj_beam.size() << std::endl;
                for( varset_beam_t::const_iterator it = adj_beam.begin(); it != adj_beam.end(); ++it ) {
                    std::map<dai::Var, size_t> state = dai::calcState(adj_beam.varset(), *it);
                    std::cout << "adj-beam: ";
                    for( std::map<dai::Var, size_t>::const_iterator jt = state.begin(); jt != state.end(); ++jt )
                        std::cout << jt->first << "=" << jt->second << ",";
                    std::cout << " index=" << *it << std::endl;
                }
#endif
        }
#if 1
        std::cout << std::endl;
#endif

        // recover valuations in beam
        for( int i = 0; i < int(valuations.size()); ++i ) {
            int valuation = valuations[i].first;
            beam.insert(valuation);
        }
        affected_beams_.insert(beam.loc());
    }

    void recover_valuations(varset_beam_t &beam, int last_action, int obs) {
        // calculate pairs of variable/values in beam (used to compute scores)
        std::set<std::pair<int, int> > pairs;
        calculate_pairs(beam, pairs);

        // calculate valuations to recover (using scores)
        assert(beam.size() <= beam.max_value());
        assert(beam.size() < beam.max_value());
        const dai::VarSet &varset = beam.varset();
        std::vector<std::pair<int, int> > best_valuations;
        float best_expected_cost = std::numeric_limits<float>::max();
        for( int valuation = 0; valuation < beam.max_value(); ++valuation ) {
            int slabels = get_slabels(beam.loc(), varset, valuation);
            float p = base_->probability_obs_ore_slam(obs, beam.loc(), slabels, last_action);
            if( beam.contains(valuation) ) continue;
            int cost = calculate_cost(valuation, beam, pairs);
            float expected_cost = (1.0 - p) * cost;
            if( expected_cost <= best_expected_cost ) {
                if( expected_cost < best_expected_cost ) {
                    best_valuations.clear();
                    best_expected_cost = expected_cost;
                }
                best_valuations.push_back(std::make_pair(valuation, cost));
            }
        }
        assert(!best_valuations.empty());

#if 1
        for( int i = 0; i < int(best_valuations.size()); ++i ) {
            int valuation = best_valuations[i].first;
            std::map<dai::Var, size_t> state = dai::calcState(beam.varset(), valuation);
            std::cout << "best: valuation={";
            for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it )
                std::cout << it->first << "=" << it->second << ",";
            std::cout << "}, index=" << valuation << std::endl;
        }
#endif

        // recover marked valuations
        affected_beams_.clear();
#ifndef NDEBUG
        int old_beam_size = beam.size();
#endif
        //std::cout << "max-value=" << beam.max_value() << std::endl;
        std::cout << "#valuations-to-recover=" << best_valuations.size() << std::endl;
        //std::cout << "size-before-recover=" << beam.size() << std::endl;
        recover_valuations(beam, best_valuations);
        //std::cout << "size-after--recover=" << beam.size() << std::endl;
        assert(old_beam_size + int(best_valuations.size()) == beam.size());

        // revise constant alpha for affected beams
        for( std::set<int>::const_iterator it = affected_beams_.begin(); it != affected_beams_.end(); ++it )
            csp_.set_alpha(*it);
    }

    void update_factors(int last_action, int obs) {
        int current_loc = loc_history_.back();
#ifdef DEBUG
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        csp_.print_factor(std::cout, current_loc);
#endif
        assert(current_loc < int(csp_.nvars()));
        std::vector<int> indices_to_be_removed;
        varset_beam_t &beam = *csp_.domain(current_loc);
        const dai::VarSet &varset = beam.varset();
        for( varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            int value = *it;
            int index = it.index();
            int slabels = get_slabels(current_loc, varset, value);
            float p = base_->probability_obs_ore_slam(obs, current_loc, slabels, last_action);
            if( p < varset_beam_t::kappa() ) indices_to_be_removed.push_back(index);
        }
#ifdef DEBUG
        std::cout << "#indices-to-be-removed=" << indices_to_be_removed.size() << std::endl;
#endif

        assert(!beam.empty());
        if( beam.size() == int(indices_to_be_removed.size()) ) {
            // Observation is inconsistent with current beam.
            // Repopulate beam with least-costly consistent valuation

#ifndef NDEBUG
            {
                std::cout << "********** INCONSISTENT OBS! *************" << std::endl;
                assert(csp_.is_consistent(0));
                csp_.add_all_edges_to_worklist();
                bool something_removed = csp_.ac3(true);
                assert(!something_removed);
            }
#endif

            if( beam.size() == beam.max_value() ) {
                std::cout << "**** BEAM IS FULL ****" << std::endl;
            } else {
                recover_valuations(beam, last_action, obs);
                for( std::set<int>::const_iterator it = affected_beams_.begin(); it != affected_beams_.end(); ++it )
                    csp_.add_to_worklist(*it);
                bool something_removed = csp_.ac3(true);

#ifndef NDEBUG
                std::cout << "ac3: something-removed=" << something_removed << ", sizes:";
                for( int i = 0; i < int(csp_.nvars()); ++i ) std::cout << " " << csp_.domain(i)->size();
                std::cout << std::endl;
                assert(!something_removed);
#endif
            }
        } else {
            beam.erase_ordered_indices(indices_to_be_removed);
            csp_.add_to_worklist(current_loc);
            csp_.add_all_edges_to_worklist();
            csp_.ac3(true);
        }
        assert(csp_.is_consistent(0));
#ifdef DEBUG
        std::cout << "factor after  update: loc=" << current_loc << ", obs=" << obs << std::endl;
        csp_.print_factor(std::cout, current_loc);
#endif
    }

    void update_marginals(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            const varset_beam_t &beam = *csp_.domain(loc);
            for( int label = 0; label < base_->nlabels_; ++label )
                marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * beam.marginal(label));
        }
        int current_loc = loc_history_.back();
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
    }

    int value_for(int /*var*/) const { return -1; }

    void print(std::ostream &os) const {
        if( loc_history_.empty() )
            os << "loc=<empty history>";
        else
            os << "loc=" << loc_history_.back();
    }
};

// Particle for the motion model RBPF filter (slam3)
struct motion_model_rbpf_slam3_particle_t : public rbpf_slam3_particle_t {

    motion_model_rbpf_slam3_particle_t() : rbpf_slam3_particle_t() { }
    motion_model_rbpf_slam3_particle_t(const motion_model_rbpf_slam3_particle_t &p)
      : rbpf_slam3_particle_t(p) {
    }
    motion_model_rbpf_slam3_particle_t(motion_model_rbpf_slam3_particle_t &&p)
      : rbpf_slam3_particle_t(std::move(p)) {
    }
    ~motion_model_rbpf_slam3_particle_t() { }

    const motion_model_rbpf_slam3_particle_t& operator=(const motion_model_rbpf_slam3_particle_t &p) {
        *static_cast<rbpf_slam3_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const motion_model_rbpf_slam3_particle_t &p) const {
        return *static_cast<const rbpf_slam3_particle_t*>(this) == p;
    }

    static std::string type() {
        return std::string("mm_rbpf3_sir");
    }

    virtual bool sample_from_pi(rbpf_slam3_particle_t &np,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                mpi_slam_t *mpi,
                                int wid) const {
#ifdef DEBUG
        assert(np == *this);
#endif
        int next_loc = base_->sample_loc(loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        if( !history_container.contains(np.loc_history_) ) {
            // this is a new loc history, perform update
            np.update_factors(last_action, obs);
        }
        return true; // CHECK: what happens with incompatible obs?
    }

    virtual float importance_weight(const rbpf_slam3_particle_t &np, int last_action, int obs) const {
        int np_current_loc = np.loc_history_.back();
        float weight = 0;
        const varset_beam_t &beam = *csp_.domain(np_current_loc);
        const dai::VarSet &varset = beam.varset();
        for( int value = 0; value < int(varset.nrStates()); ++value ) {
            int slabels = get_slabels(np_current_loc, varset, value);
            weight += beam.probability(value) * base_->probability_obs_ore_slam(obs, np_current_loc, slabels, last_action);
        }
        return weight;
    }

    motion_model_rbpf_slam3_particle_t* initial_sampling(mpi_slam_t *mpi, int wid) {
        motion_model_rbpf_slam3_particle_t *p = new motion_model_rbpf_slam3_particle_t;
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

// Particle for the optimal RBPF filter (verified: 09/12/2015)
struct optimal_rbpf_slam3_particle_t : public rbpf_slam3_particle_t {
    mutable std::vector<float> cdf_;

#if 0 // for some reason, it runs faster without these...
    optimal_rbpf_slam3_particle_t() : rbpf_slam3_particle_t() { }
    ~optimal_rbpf_slam3_particle_t() { }

    optimal_rbpf_slam3_particle_t(const optimal_rbpf_slam3_particle_t &p) {
        *this = p;
    }

    optimal_rbpf_slam3_particle_t(optimal_rbpf_slam3_particle_t &&p) : rbpf_slam3_particle_t(std::move(p)) {
    }

    const optimal_rbpf_slam3_particle_t& operator=(const optimal_rbpf_slam3_particle_t &p) {
        *static_cast<rbpf_slam3_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const optimal_rbpf_slam3_particle_t &p) const {
        return *static_cast<const rbpf_slam3_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("opt_rbpf3_sir");
    }

    void calculate_cdf(int last_action, int obs, std::vector<float> &cdf) const {
        // make sure there is no pending inference on factor model
 #if 0
        assert(indices_for_updated_factors_.empty());

        cdf.clear();
        cdf.reserve(base_->nloc_);

        // P(nloc | loc, action, obs) = alpha * P(nloc, obs | loc, action)
        //                            = alpha * P(obs | nloc, loc, action) * P(nloc | loc, action)
        //                            = alpha * P(obs | nloc, action) * P(nloc | loc, action)

        float previous = 0;
        int current_loc = loc_history_.back();
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            const dai::Factor &p_marginal = marginals_[nloc];
            float prob = 0;
            for( int value = 0; value < int(p_marginal.nrStates()); ++value ) {
                int slabels = get_slabels(nloc, p_marginal.vars(), value);
                prob += p_marginal[value] * base_->probability_obs_ore_slam(obs, nloc, slabels, last_action);
            }
            cdf.push_back(previous + base_->probability_tr_loc(last_action, current_loc, nloc) * prob);
            previous = cdf.back();
        }

        // normalize (i.e. calculate alpha)
        assert(cdf.back() > 0);
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            cdf[nloc] /= cdf.back();
        }
#endif
        assert(0);
    }

    virtual bool sample_from_pi(rbpf_slam3_particle_t &np,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                mpi_slam_t *mpi,
                                int wid) const {
#ifdef DEBUG
        assert(*this == np);
#endif
        calculate_cdf(last_action, obs, cdf_);
        int next_loc = Utils::sample_from_distribution(base_->nloc_, &cdf_[0]);
        np.loc_history_.push_back(next_loc);
        if( !history_container.contains(np.loc_history_) ) {
            // this is a new loc history, perform update
            np.update_factors(last_action, obs);
        }
        return true; // CHECK: what happens with incompatible obs?
    }

    virtual float importance_weight(const rbpf_slam3_particle_t &, int last_action, int obs) const {
        float weight = 0;
        int current_loc = loc_history_.back();
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            const varset_beam_t &beam = *csp_.domain(current_loc);
            const dai::VarSet &varset = beam.varset();
            float p = 0;
            for( int value = 0; value < int(varset.nrStates()); ++value ) {
                int slabels = get_slabels(current_loc, varset, value);
                p += beam.probability(value) * base_->probability_obs_ore_slam(obs, nloc, slabels, last_action);
            }
            weight += base_->probability_tr_loc(last_action, current_loc, nloc) * p;
        }
        return weight;
    }

    optimal_rbpf_slam3_particle_t* initial_sampling(mpi_slam_t *mpi, int wid) {
        optimal_rbpf_slam3_particle_t *p = new optimal_rbpf_slam3_particle_t;
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

#undef DEBUG

#endif

