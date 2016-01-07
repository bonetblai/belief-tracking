/*
 *  Copyright (C) 2016 Universidad Simon Bolivar
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

#ifndef SLAM4_PARTICLES_H
#define SLAM4_PARTICLES_H

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
#include "kappa.h"
#include "utils.h"

#include "weighted_var_beam.h"
#include "weighted_arc_consistency.h"

//#define DEBUG

namespace slam4 {

class cache_t {
    static int num_locs_;
    static std::vector<dai::Var> variables_;
    static std::vector<dai::VarSet> varsets_;
    static std::vector<unsigned char> compatible_values_;
    static std::vector<std::vector<std::map<dai::Var, size_t>*> > state_cache_;

    static void compute_basic_elements(int nrows, int ncols) {
        // variables for each location
        variables_ = std::vector<dai::Var>(num_locs_);
        for( int loc = 0; loc < num_locs_; ++loc )
            variables_[loc] = dai::Var(loc, 2);

        // variable sets for each location
        varsets_ = std::vector<dai::VarSet>(num_locs_);
        for( int loc = 0; loc < num_locs_; ++loc ) {
            int row = loc / ncols, col = loc % ncols;
            std::vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= nrows) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= ncols) ) continue;
                    vars.push_back(variables_[nr * ncols + nc]);
                }
            }
            std::sort(vars.begin(), vars.end());
            varsets_[loc] = dai::VarSet(vars.begin(), vars.end());
#ifdef DEBUG
            std::cout << "# compute_basic_elements: loc=" << loc << ", vars=" << varsets_[loc] << std::endl;
#endif
        }
    }

    static void compute_cache_for_states(int nrows, int ncols) {
        float start_time = Utils::read_time_in_seconds();
        int size = 0;
        state_cache_ = std::vector<std::vector<std::map<dai::Var, size_t>*> >(num_locs_);
        for( int loc = 0; loc < num_locs_; ++loc ) {
            const dai::VarSet &varset = varsets_[loc];
            state_cache_[loc] = std::vector<std::map<dai::Var, size_t>*>(varset.nrStates(), static_cast<std::map<dai::Var, size_t>*>(0));
            for( int value = 0; value < int(varset.nrStates()); ++value ) {
                std::map<dai::Var, size_t> *state = new std::map<dai::Var, size_t>(dai::calcState(varset, value));
                state_cache_[loc][value] = state;
                ++size;
            }
        }
        std::cout << "# cache-for-states:"
                  << " size=" << size
                  //<< ", hits=" << cache_hits
                  << ", time=" << Utils::read_time_in_seconds() - start_time
                  << std::endl;
    }

    static void compute_cache_for_compatible_values(int nrows, int ncols) {
        float start_time = Utils::read_time_in_seconds();

        // for each location (var_y) and value for it, for each adjacent loc in constraint graph,
        // determine compatible values
        std::map<std::vector<int>, unsigned char> cache;
        unsigned cache_hits = 0;
        compatible_values_ = std::vector<unsigned char>(num_locs_ * num_locs_ * 8, static_cast<unsigned char>(0));
        for( int var_y = 0; var_y < num_locs_; ++var_y ) {
            int col = var_y % ncols;
            for( int val_y = 0; val_y < int(varsets_[var_y].nrStates()); ++val_y ) {
                const std::map<dai::Var, size_t> &state_y = *state_cache_[var_y][val_y];
                for( int dc = -2; dc < 3; ++dc ) {
                    int var_x = col + dc;
                    if( (var_x < 0) || (var_x >= ncols) ) continue;

                    // check whether we have already calculated this
                    std::vector<int> cache_key;
                    for( int dc2 = -2; dc2 < 3; ++dc2 ) {
                        int nc2 = col + dc2;
                        if( (nc2 < 0) || (nc2 >= ncols) )
                            cache_key.push_back(-1);
                        else
                            cache_key.push_back(varsets_[nc2].size());
                    }
                    cache_key.push_back(val_y);
                    cache_key.push_back(dc);
                    std::map<std::vector<int>, unsigned char>::const_iterator cache_it = cache.find(cache_key);
                    if( cache_it != cache.end() ) {
                        ++cache_hits;
                        assert(val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x < int(compatible_values_.size()));
                        compatible_values_[val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x] = cache_it->second;
                    } else {
                        unsigned char values = 0;
                        int ncompatible = 0;
if( var_y == 0 && var_x == 2 && val_y == 1 ) std::cout << "var-y=" << var_y << ", val-y=" << val_y << ", var-x=" << var_x << ":";
                        for( int val_x = 0; val_x < int(varsets_[var_x].nrStates()); ++val_x ) {
                            bool compatible = true;
                            const std::map<dai::Var, size_t> &state_x = *state_cache_[var_x][val_x];
                            for( std::map<dai::Var, size_t>::const_iterator it = state_x.begin(); it != state_x.end(); ++it ) {
                                std::map<dai::Var, size_t>::const_iterator jt = state_y.find(it->first);
                                if( (jt != state_y.end()) && (jt->second != it->second) ) {
                                    compatible = false;
                                    break;
                                }
                            }
                            if( compatible ) {
if( var_y == 0 && var_x == 2 && val_y == 1 ) std::cout << " " << val_x;
                                assert(val_x < 8);
                                int offset = val_x % 8;
                                values |= 1 << offset;
                                ++ncompatible;
                            }
                        }
if( var_y == 0 && var_x == 2 && val_y == 1 ) std::cout << " values=" << unsigned(values) << std::endl;
                        assert((ncompatible > 0) && (ncompatible < int(varsets_[var_x].nrStates())));
                        assert(val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x < int(compatible_values_.size()));
                        compatible_values_[val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x] = values;
                        //cache.insert(std::make_pair(cache_key, values));
                    }
                }
            }
        }
        std::cout << "# cache-for-compatible-values:"
                  << " size=" << cache.size()
                  << ", hits=" << cache_hits
                  << ", time=" << Utils::read_time_in_seconds() - start_time
                  << std::endl;
    }

  public:
    cache_t() { }
    ~cache_t() { }

    static void initialize(int nrows, int ncols) {
        num_locs_ = nrows * ncols;
        compute_basic_elements(nrows, ncols);
        compute_cache_for_states(nrows, ncols);
        compute_cache_for_compatible_values(nrows, ncols);
    }

    static const dai::VarSet& varset(int loc) {
        return varsets_[loc];
    }
    static unsigned char compatible_values(int val_y, int var_y, int var_x) {
        return compatible_values_[val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x];
    }
    static std::map<dai::Var, size_t>& state(int var, int value) {
        return *state_cache_[var][value];
    }

    static void print_state(std::ostream &os, const std::map<dai::Var, size_t> &state) {
        os << "{";
        for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it )
            os << it->first << "=" << it->second << ",";
        os << "}";
    }
};

class weighted_varset_beam_t : public weighted_var_beam_t {
  protected:
    const int loc_;
    const dai::Var var_;
    const dai::VarSet varset_;
    unsigned mask_;
    float normalization_constant_;

  public:
    weighted_varset_beam_t(int loc, const dai::Var &var, const dai::VarSet &varset)
      : weighted_var_beam_t(1, varset.nrStates()), loc_(loc), var_(var), varset_(varset), mask_(0), normalization_constant_(1) {
        mask_ = unsigned(-1);
        for( dai::State state(varset_); state.valid(); state++ ) {
            if( state(var_) == 1 )
                mask_ = mask_ & dai::calcLinearState(varset_, state);
        }
#ifdef DEBUG
        std::cout << "weighted_varset_beam_t: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    weighted_varset_beam_t(const weighted_varset_beam_t &beam)
      : weighted_var_beam_t(beam),
        loc_(beam.loc_),
        var_(beam.var_),
        varset_(beam.varset_),
        mask_(beam.mask_),
        normalization_constant_(beam.normalization_constant_) {
#ifdef DEBUG
        std::cout << "weighted_varset_beam_t: copy-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    weighted_varset_beam_t(weighted_varset_beam_t &&beam)
      : weighted_var_beam_t(std::move(beam)),
        loc_(beam.loc_),
        var_(beam.var_),
        varset_(std::move(beam.varset_)),
        mask_(beam.mask_),
        normalization_constant_(beam.normalization_constant_) {
#ifdef DEBUG
        std::cout << "weighted_varset_beam_t: move-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    ~weighted_varset_beam_t() { }

    bool operator==(const weighted_varset_beam_t &beam) const {
        return (loc_ == beam.loc_) && (*static_cast<const weighted_var_beam_t*>(this) == *static_cast<const weighted_var_beam_t*>(&beam));
    }

    int loc() const { return loc_; }
    const dai::VarSet& varset() const { return varset_; }

    float probability(unsigned valuation, int weight) const {
        return weight == std::numeric_limits<int>::max() ? 0 : normalization_constant_ * kappa_t::power(weight);
    }
    float probability(int valuation) const {
        int w = weight(valuation);
        return w == -1 ? 0 : probability(valuation, w);
    }
    float marginal(int label) const {
        float p = 0;
        for( const_iterator it = begin(); it != end(); ++it ) {
            unsigned valuation = *it;
            if( ((label == 1) && ((valuation & mask_) != 0)) || ((label == 0) && ((valuation & mask_) == 0)) )
                p += probability(valuation, it.weight());
        }
        return p;
    }

    void calculate_normalization_constant() {
        float mass = 0;
        for( const_iterator it = begin(); it != end(); ++it )
            mass += kappa_t::power(it.weight());
        normalization_constant_ = 1.0 / mass;
    }
    void set_initial_configuration() {
        weighted_var_beam_t::set_initial_configuration();
        calculate_normalization_constant();
    }
    void erase_ordered_indices(const std::vector<int> &ordered_indices) {
        weighted_var_beam_t::erase_ordered_indices(ordered_indices);
    }
};

class weighted_arc_consistency_t : public CSP::weighted_arc_consistency_t<weighted_varset_beam_t> {
    static CSP::constraint_digraph_t cg_;
    mutable std::map<dai::Var, size_t> *state_x_;
    mutable unsigned char bitmask_;

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

    virtual void arc_reduce_preprocessing_1(int var_x, int val_x) const {
        state_x_ = &cache_t::state(var_x, val_x);
    }
    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const {
        const std::map<dai::Var, size_t> &state_y = cache_t::state(var_y, val_y);
        for( std::map<dai::Var, size_t>::const_iterator it = state_x_->begin(); it != state_x_->end(); ++it ) {
            std::map<dai::Var, size_t>::const_iterator jt = state_y.find(it->first);
            if( (jt != state_y.end()) && (it->second != jt->second) ) return false;
        }
        return true;
    }

    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y) const {
        bitmask_ = 0;
    }
    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y, int val_y) const {
        bitmask_ |= cache_t::compatible_values(val_y, var_y, var_x);
    }
    virtual bool arc_reduce_inverse_check(int val_x) const {
        int offset = val_x % 8;
        return ((bitmask_ >> offset) & 0x1) == 1;
    }

  public:
    weighted_arc_consistency_t() : CSP::weighted_arc_consistency_t<weighted_varset_beam_t>(cg_) { }
    weighted_arc_consistency_t(const weighted_arc_consistency_t &ac) = delete;
    virtual ~weighted_arc_consistency_t() { }

    static void initialize(int nrows, int ncols) {
        construct_constraint_graph(nrows, ncols);
    }

    const weighted_arc_consistency_t& operator=(const weighted_arc_consistency_t &ac) {
        nvars_ = ac.nvars_;
        assert(domain_.size() == ac.domain_.size());
        for( int loc = 0; loc < int(ac.domain_.size()); ++loc )
            set_domain(loc, new weighted_varset_beam_t(*ac.domain(loc)));
        return *this;
    }
    bool operator==(const weighted_arc_consistency_t &ac) const {
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

    bool weighted_ac3() {
        std::vector<int> revised_vars;
        bool something_removed = CSP::weighted_arc_consistency_t<weighted_varset_beam_t>::weighted_ac3(revised_vars, true);
        normalize_weights(revised_vars);
        calculate_normalization_constant(revised_vars);
        return something_removed;
    }

    void calculate_normalization_constant(int loc) {
        domain_[loc]->calculate_normalization_constant();
    }
    void calculate_normalization_constant(const std::vector<int> &revised_vars) {
        for( int i = 0; i < int(revised_vars.size()); ++i )
            calculate_normalization_constant(revised_vars[i]);
    }

    void normalize_weights(const std::vector<int> &revised_vars) {
        for( int i = 0; i < int(revised_vars.size()); ++i )
            domain_[revised_vars[i]]->normalize();
    }
    bool normalized_weights() {
        for( int i = 0; i < int(domain_.size()); ++i ) {
            if( !domain_[i]->normalized() )
                return false;
        }
        return true;
    }

    void print_factor(std::ostream &os, int loc) const {
        const weighted_varset_beam_t &beam = *domain_[loc];
        os << "loc=" << loc << ",sz=" << beam.size() << ",domain=" << beam;
    }
    void print(std::ostream &os) const {
        for( int loc = 0; loc < int(domain_.size()); ++loc ) {
            os << "[";
            print_factor(os, loc);
            os << "]" << std::endl;
        }
    }
};

}; // namespace slam4

// Abstract Particle for the 2nd Rao-Blackwellised filter
struct rbpf_slam4_particle_t : public base_particle_t {
    std::vector<int> loc_history_;

    const std::vector<int>& history() const {
        return loc_history_;
    }

    // arc consistency
    slam4::weighted_arc_consistency_t weighted_csp_;
    std::set<int> affected_beams_;

    // cache for conversion from factor values into slabels (it is dynamically filled by get_slabels())
    static std::vector<std::vector<int> > slabels_;

    rbpf_slam4_particle_t() {
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
            weighted_csp_.set_domain(loc, new slam4::weighted_varset_beam_t(loc, variables[loc], varset));
        }
    }
    rbpf_slam4_particle_t(const rbpf_slam4_particle_t &p)
      : loc_history_(p.loc_history_) {
        weighted_csp_ = p.weighted_csp_;
    }
#if 0
    rbpf_slam4_particle_t(rbpf_slam4_particle_t &&p)
      : loc_history_(std::move(p.loc_history_)),
        weighted_csp_(std::move(p.weighted_csp_)) {
    }
#endif
    virtual ~rbpf_slam4_particle_t() {
        weighted_csp_.delete_domains_and_clear();
    }

    const rbpf_slam4_particle_t& operator=(const rbpf_slam4_particle_t &p) {
        loc_history_ = p.loc_history_;
        weighted_csp_ = p.weighted_csp_;
        return *this;
    }

    bool operator==(const rbpf_slam4_particle_t &p) const {
        return (loc_history_ == p.loc_history_) && (weighted_csp_ == p.weighted_csp_);
    }

    void reset_csp() {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            weighted_csp_.domain(loc)->set_initial_configuration();
        }
    }

    void initial_sampling_in_place() {
        assert(base_->nlabels_ == 2);
        loc_history_.push_back(base_->initial_loc_);
        reset_csp();
        assert(weighted_csp_.is_consistent(0));
    }

    int get_slabels(int loc, const dai::VarSet &varset, int value) const {
        assert((value >= 0) && (value < int(varset.nrStates())));

        // allocate cache if this is first call
        if( slabels_.empty() )
            slabels_ = std::vector<std::vector<int> >(base_->nloc_, std::vector<int>(8, -1));
        assert(loc < int(slabels_.size()));
        if( slabels_[loc].empty() )
            slabels_[loc] = std::vector<int>(varset.nrStates(), -1);
        assert(value < int(slabels_[loc].size()));

        // check whether there is a valid entry in cache
        const std::vector<int> &slabels_for_loc = slabels_[loc];
        if( slabels_for_loc[value] != -1 )
            return slabels_for_loc[value];

#ifdef DEBUG
        std::cout << "get_slabels(loc=" << loc << ":" << coord_t(loc) << ", value=" << value << "):" << std::endl;
#endif

        // this is the first time that we access (beam,value)
        // compute the correct value and cache it for later use
        int slabels = 0;
        const std::map<dai::Var, size_t> &state = slam4::cache_t::state(loc, value);
        for( dai::VarSet::const_iterator it = varset.begin(); it != varset.end(); ++it ) {
            const dai::Var &var = *it;
            std::map<dai::Var, size_t>::const_iterator jt = state.find(var);
            assert(jt != state.end());
            size_t var_value = jt->second;
            assert(var_value < 2); // because it is a binary variable
            if( var_value ) {
                int var_id = it->label();
                assert((loc - var_id > -2) || (loc - var_id < 2));
                int var_off = 1 + var_id - loc;
                assert((var_off >= 0) && (var_off < 3));
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
    int get_slabels(const slam4::weighted_varset_beam_t &beam, int value) const {
        return get_slabels(beam.loc(), beam.varset(), value);
    }

    bool update_factors(int last_action, int obs) {
        int current_loc = loc_history_.back();
#ifdef DEBUG
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  weighted-csp: ";
        weighted_csp_.print_factor(std::cout, current_loc);
        std::cout << std::endl;
#endif
        assert(current_loc < int(weighted_csp_.nvars()));
        std::vector<std::pair<int, int> > weight_increases;
        slam4::weighted_varset_beam_t &beam = *weighted_csp_.domain(current_loc);
        for( slam4::weighted_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            int value = *it;
            int index = it.index();
            int slabels = get_slabels(beam, value);
            float p = base_->probability_obs(obs, current_loc, slabels, last_action);
            int kappa_obs = kappa_t::kappa(p);
            weight_increases.push_back(std::make_pair(index, kappa_obs));
        }
#ifdef DEBUG
        std::cout << "  increases:";
        for( int i = 0; i < int(weight_increases.size()); ++i ) {
            std::pair<int, int> &p = weight_increases[i];
            int valuation = beam[p.first].first;
            int weight = beam[p.first].second;
            std::cout << " {" << p.first << ":" << valuation << ":";
            std::map<dai::Var, size_t> state = dai::calcState(beam.varset(), valuation);
            for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it )
                std::cout << it->first << "=" << it->second << ",";
            std::cout << "weight=" << weight << ",amount=" << p.second << "},";
        }
        std::cout << std::endl;
#endif

        assert(!beam.empty());
        for( int i = 0; i < int(weight_increases.size()); ++i )
            weighted_csp_.domain(current_loc)->increase_weight(weight_increases[i]);
        weighted_csp_.add_to_worklist(current_loc);
        weighted_csp_.weighted_ac3();

#ifdef DEBUG
        std::cout << "factor after update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  weighted-csp: ";
        weighted_csp_.print_factor(std::cout, current_loc);
        std::cout << std::endl;
#endif
        return weighted_csp_.is_consistent(0);
    }

    void update_marginals(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            const slam4::weighted_varset_beam_t &beam = *weighted_csp_.domain(loc);
            for( int label = 0; label < base_->nlabels_; ++label )
                marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * beam.marginal(label));
        }
        int current_loc = loc_history_.back();
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
    }

    int value_for(int /*var*/) const { return -1; }

    void initialize_mpi_worker(mpi_slam_t * /*mpi*/, int /*wid*/) { }
    void mpi_update_marginals(mpi_slam_t * /*mpi*/, int /*wid*/) { }

    void print(std::ostream &os) const {
        if( loc_history_.empty() )
            os << "loc=<empty history>";
        else
            os << "loc=" << loc_history_.back();
    }
};

// Particle for the motion model RBPF filter (slam4)
struct motion_model_rbpf_slam4_particle_t : public rbpf_slam4_particle_t {

    motion_model_rbpf_slam4_particle_t() : rbpf_slam4_particle_t() { }
    motion_model_rbpf_slam4_particle_t(const motion_model_rbpf_slam4_particle_t &p)
      : rbpf_slam4_particle_t(p) {
    }
    motion_model_rbpf_slam4_particle_t(motion_model_rbpf_slam4_particle_t &&p)
      : rbpf_slam4_particle_t(std::move(p)) {
    }
    ~motion_model_rbpf_slam4_particle_t() { }

    const motion_model_rbpf_slam4_particle_t& operator=(const motion_model_rbpf_slam4_particle_t &p) {
        *static_cast<rbpf_slam4_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const motion_model_rbpf_slam4_particle_t &p) const {
        return *static_cast<const rbpf_slam4_particle_t*>(this) == p;
    }

    static std::string type() {
        return std::string("mm_rbpf4_sir");
    }

    virtual bool sample_from_pi(rbpf_slam4_particle_t &np,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                mpi_slam_t * /*mpi*/,
                                int wid) const {
#ifdef DEBUG
        assert(np == *this);
#endif
        int next_loc = base_->sample_loc(loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        if( !history_container.contains(np.loc_history_) ) {
            // this is a new loc history, perform update
            return np.update_factors(last_action, obs);
        }
        return true;
    }

    virtual float importance_weight(const rbpf_slam4_particle_t &np, int last_action, int obs) const {
        int np_current_loc = np.loc_history_.back();
        float weight = 0;
        const slam4::weighted_varset_beam_t &beam = *weighted_csp_.domain(np_current_loc);
        const dai::VarSet &varset = beam.varset();
        for( int value = 0; value < int(varset.nrStates()); ++value ) {
            int slabels = get_slabels(beam, value);
            weight += beam.probability(value) * base_->probability_obs(obs, np_current_loc, slabels, last_action);
        }
        return weight;
    }

    motion_model_rbpf_slam4_particle_t* initial_sampling(mpi_slam_t * /*mpi*/, int wid) {
        motion_model_rbpf_slam4_particle_t *p = new motion_model_rbpf_slam4_particle_t;
        p->initial_sampling_in_place();
        return p;
    }
};

// Particle for the optimal RBPF filter (verified: 09/12/2015)
struct optimal_rbpf_slam4_particle_t : public rbpf_slam4_particle_t {
    mutable std::vector<float> cdf_;

#if 0 // for some reason, it runs faster without these...
    optimal_rbpf_slam4_particle_t() : rbpf_slam4_particle_t() { }
    ~optimal_rbpf_slam4_particle_t() { }

    optimal_rbpf_slam4_particle_t(const optimal_rbpf_slam4_particle_t &p) {
        *this = p;
    }

    optimal_rbpf_slam4_particle_t(optimal_rbpf_slam4_particle_t &&p) : rbpf_slam4_particle_t(std::move(p)) {
    }

    const optimal_rbpf_slam4_particle_t& operator=(const optimal_rbpf_slam4_particle_t &p) {
        *static_cast<rbpf_slam4_particle_t*>(this) = p;
        return *this;
    }

    bool operator==(const optimal_rbpf_slam4_particle_t &p) const {
        return *static_cast<const rbpf_slam4_particle_t*>(this) == p;
    }
#endif

    static std::string type() {
        return std::string("opt_rbpf4_sir");
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
                prob += p_marginal[value] * base_->probability_obs(obs, nloc, slabels, last_action);
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

    virtual bool sample_from_pi(rbpf_slam4_particle_t &np,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                mpi_slam_t * /*mpi*/,
                                int wid) const {
#ifdef DEBUG
        assert(*this == np);
#endif
        calculate_cdf(last_action, obs, cdf_);
        int next_loc = Utils::sample_from_distribution(base_->nloc_, &cdf_[0]);
        np.loc_history_.push_back(next_loc);
        if( !history_container.contains(np.loc_history_) ) {
            // this is a new loc history, perform update
            return np.update_factors(last_action, obs);
        }
        return true;
    }

    virtual float importance_weight(const rbpf_slam4_particle_t &, int last_action, int obs) const {
        float weight = 0;
        int current_loc = loc_history_.back();
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            const slam4::weighted_varset_beam_t &beam = *weighted_csp_.domain(current_loc);
            const dai::VarSet &varset = beam.varset();
            float p = 0;
            for( int value = 0; value < int(varset.nrStates()); ++value ) {
                int slabels = get_slabels(beam, value);
                p += beam.probability(value) * base_->probability_obs(obs, nloc, slabels, last_action);
            }
            weight += base_->probability_tr_loc(last_action, current_loc, nloc) * p;
        }
        return weight;
    }

    optimal_rbpf_slam4_particle_t* initial_sampling(mpi_slam_t * /*mpi*/, int wid) {
        optimal_rbpf_slam4_particle_t *p = new optimal_rbpf_slam4_particle_t;
        p->initial_sampling_in_place();
        return p;
    }
};

#undef DEBUG

#endif

