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

#ifndef ORE_SLAM_PARTICLES_H
#define ORE_SLAM_PARTICLES_H

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
#include "slam_cache.h"
#include "inference.h"
#include "edbp.h"
#include "kappa.h"
#include "utils.h"

#include "weighted_var_beam.h"
#include "weighted_arc_consistency.h"

//#define DEBUG

namespace OreSLAM {

class cache_t : public SLAM::cache_t {
  protected:
    static std::vector<unsigned*> compatible_values_;

    static void compute_cache_for_compatible_values(int nrows, int ncols) {
        float start_time = Utils::read_time_in_seconds();

        // for each location (var_y) and value for it, for each adjacent loc in constraint graph,
        // determine compatible values
        std::map<std::string, unsigned*> cache;
        unsigned cache_hits = 0;
        compatible_values_ = std::vector<unsigned*>(num_locs_ * num_locs_ * 512, static_cast<unsigned*>(0));
        for( int var_y = 0; var_y < num_locs_; ++var_y ) {
            int row = var_y / ncols, col = var_y % ncols;
            for( int val_y = 0; val_y < int(varsets_[var_y].nrStates()); ++val_y ) {
                const std::map<dai::Var, size_t> &state_y = *state_cache_[var_y][val_y];
                for( int dc = -2; dc < 3; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= ncols) ) continue;
                    for( int dr = -2; dr < 3; ++dr ) {
                        int nr = row + dr;
                        if( (nr < 0) || (nr >= nrows) ) continue;
                        int var_x = nr * ncols + nc;
                        if( var_x == var_y ) continue;

                        // check whether we have already calculated this entry
                        std::string cache_key;
                        cache_key += std::to_string(varsets_[var_y].size()) + " ";
                        cache_key += std::to_string(varsets_[var_x].size()) + " ";
                        cache_key += std::to_string(dr) + " ";
                        cache_key += std::to_string(dc) + " ";
                        cache_key += std::to_string(val_y) + " ";
                        cache_key += std::to_string(row == 0);
                        cache_key += std::to_string(row == nrows - 1);
                        cache_key += std::to_string(col == 0);
                        cache_key += std::to_string(col == ncols - 1);
                        cache_key += std::to_string(nr == 0);
                        cache_key += std::to_string(nr == nrows - 1);
                        cache_key += std::to_string(nc == 0);
                        cache_key += std::to_string(nc == ncols - 1);
                        std::map<std::string, unsigned*>::const_iterator cache_it = cache.find(cache_key);
                        if( cache_it != cache.end() ) {
                            ++cache_hits;
                            assert(val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x < int(compatible_values_.size()));
                            compatible_values_[val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x] = cache_it->second;
                        } else {
                            int size = int(varsets_[var_x].nrStates()) >> 5;
                            size = size == 0 ? 1 : size;
                            unsigned *values = new unsigned[size]();
                            int ncompatible = 0;
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
                                    int index = val_x / 32, offset = val_x % 32;
                                    assert(index < size);
                                    values[index] |= 1 << offset;
                                    ++ncompatible;
                                }
                            }
                            assert((ncompatible > 0) && (ncompatible < int(varsets_[var_x].nrStates())));
                            assert(val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x < int(compatible_values_.size()));
                            compatible_values_[val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x] = values;
                            cache.insert(std::make_pair(cache_key, values));
                        }
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
        SLAM::cache_t::compute_basic_elements(nrows, ncols);
        SLAM::cache_t::compute_cache_for_states(nrows, ncols);
        compute_cache_for_compatible_values(nrows, ncols);
    }
    static void finalize() {
        std::set<unsigned*> erased;
        for( int i = 0; i < int(compatible_values_.size()); ++i ) {
            unsigned *values = compatible_values_[i];
            if( (values != 0) && (erased.find(values) == erased.end()) ) {
                delete[] values;
                erased.insert(values);
            }
        }
        SLAM::cache_t::finalize();
    }

    static const unsigned* compatible_values(int val_y, int var_y, int var_x) {
        return compatible_values_[val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x];
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
    mutable const std::map<dai::Var, size_t> *state_x_;
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
        //state_x_ = &cache_t::state(var_x, val_x);
    }
    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const {
#if 0
        const std::map<dai::Var, size_t> &state_y = cache_t::state(var_y, val_y);
        for( std::map<dai::Var, size_t>::const_iterator it = state_x_->begin(); it != state_x_->end(); ++it ) {
            std::map<dai::Var, size_t>::const_iterator jt = state_y.find(it->first);
            if( (jt != state_y.end()) && (it->second != jt->second) ) {
                return false;
            }
        }
        return true;
#else
        const unsigned *values = cache_t::compatible_values(val_y, var_y, var_x);
        int index = val_x / 32, offset = val_x % 32;
        return (values[index] & (1 << offset)) != 0;
#endif
    }

#if 0
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
#endif

  public:
    weighted_arc_consistency_t() : CSP::weighted_arc_consistency_t<weighted_varset_beam_t>(cg_) { }
    weighted_arc_consistency_t(const weighted_arc_consistency_t &ac) : CSP::weighted_arc_consistency_t<weighted_varset_beam_t>(ac.cg_) { }
    weighted_arc_consistency_t(weighted_arc_consistency_t &&ac) = default;
    virtual ~weighted_arc_consistency_t() { }

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

    static void initialize(int nrows, int ncols) {
        construct_constraint_graph(nrows, ncols);
    }

    void set_max_kappa(int max_kappa) {
        CSP::weighted_arc_consistency_t<weighted_varset_beam_t>::max_allowed_weight_ = max_kappa;
    }

    bool weighted_ac3() {
        std::vector<int> revised_vars;
        bool something_removed = CSP::weighted_arc_consistency_t<weighted_varset_beam_t>::weighted_ac3(revised_vars, true);
        normalize_weights(revised_vars);
        calculate_normalization_constant(revised_vars);
        assert(normalized_weights());
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

// Abstract Particle for Rao-Blackwellised filter
struct rbpf_particle_t : public base_particle_t {
    std::vector<int> loc_history_;

    const std::vector<int>& history() const {
        return loc_history_;
    }

    // arc consistency
    bool use_ac3_;
    int max_kappa_;
    weighted_arc_consistency_t weighted_csp_;

    // factors
    std::vector<dai::Factor> factors_;

    // computation of marginals in factor model
    mutable std::vector<int> indices_for_updated_factors_;
    mutable std::vector<dai::Factor> marginals_;
    Inference::inference_t inference_;

    rbpf_particle_t(bool use_ac3, int max_kappa) : use_ac3_(use_ac3), max_kappa_(max_kappa) {
        assert(base_ != 0);
        assert(base_->nlabels_ == 2);
        if( !use_ac3_ ) {
            factors_ = std::vector<dai::Factor>(base_->nloc_);
            marginals_ = std::vector<dai::Factor>(base_->nloc_);
            weighted_csp_.delete_domains_and_clear();
        }

        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            if( use_ac3_ ) {
                weighted_csp_.set_domain(loc, new weighted_varset_beam_t(loc, cache_t::variable(loc), cache_t::varset(loc)));
            } else {
                factors_[loc] = dai::Factor(cache_t::varset(loc));
                marginals_[loc] = dai::Factor(cache_t::varset(loc));
            }
        }
        if( !use_ac3_ ) inference_.create_and_initialize_algorithm(factors_);
    }
    rbpf_particle_t(const std::multimap<std::string, std::string> &parameters) : rbpf_particle_t(false, std::numeric_limits<int>::max()) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("use-ac3");
        if( it != parameters.end() )
            use_ac3_ = it->second == "true";
        it = parameters.find("max-kappa");
        if( it != parameters.end() ) {
            max_kappa_ = strtod(it->second.c_str(), 0);
            weighted_csp_.set_max_kappa(max_kappa_);
        }
    }
    rbpf_particle_t(const rbpf_particle_t &p) {
        if( !p.use_ac3_ ) weighted_csp_.delete_domains_and_clear();
        *this = p;
    }
    rbpf_particle_t(rbpf_particle_t &&p)
      : loc_history_(std::move(p.loc_history_)),
        use_ac3_(p.use_ac3_),
        max_kappa_(p.max_kappa_),
        weighted_csp_(std::move(p.weighted_csp_)),
        factors_(std::move(p.factors_)),
        indices_for_updated_factors_(std::move(p.indices_for_updated_factors_)),
        marginals_(std::move(p.marginals_)),
        inference_(std::move(p.inference_)) {
    }
    virtual ~rbpf_particle_t() {
        weighted_csp_.delete_domains_and_clear();
        if( use_ac3_) inference_.destroy_inference_algorithm();
    }

    const rbpf_particle_t& operator=(const rbpf_particle_t &p) {
        loc_history_ = p.loc_history_;
        use_ac3_ = p.use_ac3_;
        max_kappa_ = p.max_kappa_;
        weighted_csp_ = p.weighted_csp_;
        factors_ = p.factors_;
        indices_for_updated_factors_ = p.indices_for_updated_factors_;
        marginals_ = p.marginals_;
        inference_ = p.inference_;
        return *this;
    }

    bool operator==(const rbpf_particle_t &p) const {
        return (loc_history_ == p.loc_history_) &&
               (use_ac3_ == p.use_ac3_) &&
               (max_kappa_ == p.max_kappa_) &&
               (weighted_csp_ == p.weighted_csp_) &&
               (factors_ == p.factors_) &&
               (indices_for_updated_factors_ == p.indices_for_updated_factors_) &&
               (marginals_ == p.marginals_) &&
               (inference_ == p.inference_);
    }

    void reset_csp() {
        assert(use_ac3_);
        for( int loc = 0; loc < base_->nloc_; ++loc )
            weighted_csp_.domain(loc)->set_initial_configuration();
    }

    void initialize_mpi_worker(mpi_slam_t *mpi, int wid) {
#ifdef USE_MPI
        assert(mpi != 0);

        // initialize io buffers
        if( mpi->io_buffer_ == 0 )
            mpi->initialize_buffers(factors_);

        // initialize worker 
        mpi->initialize_worker(cache_t::variables(), factors_, wid);
#endif
    }

    void initial_sampling_in_place(mpi_slam_t *mpi, int wid) {
        assert(base_->nlabels_ == 2);
        loc_history_.push_back(base_->initial_loc_);
        if( use_ac3_ ) {
            reset_csp();
        } else {
            // set initial history and reset factors for locations
            indices_for_updated_factors_.clear();
            indices_for_updated_factors_.reserve(base_->nloc_);
            for( int loc = 0; loc < base_->nloc_; ++loc ) {
                dai::Factor &factor = factors_[loc];
                float p = 1.0 / (1 << factor.vars().size());
                for( int i = 0; i < (1 << factor.vars().size()); ++i )
                    factor.set(i, p);
                indices_for_updated_factors_.push_back(loc);
            }
            calculate_marginals(mpi, wid, false);
        }
        assert(!use_ac3_ || weighted_csp_.is_consistent(0));
    }

    static int var_offset(int loc, int var_id) { return base_->var_offset(loc, var_id); }
    int get_slabels(const weighted_varset_beam_t &beam, int value) const {
        return cache_t::get_slabels(beam.loc(), beam.varset(), value, var_offset);
    }
    int get_slabels(int loc, const dai::VarSet &varset, int value) const {
        return cache_t::get_slabels(loc, varset, value, var_offset);
    }

    bool update_factors_ac3(int last_action, int obs) {
        assert(use_ac3_);
        int current_loc = loc_history_.back();
#ifdef DEBUG
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  weighted-csp: ";
        weighted_csp_.print_factor(std::cout, current_loc);
        std::cout << std::endl;
#endif
        assert(current_loc < int(weighted_csp_.nvars()));
        std::vector<std::pair<int, int> > weight_increases;
        weighted_varset_beam_t &beam = *weighted_csp_.domain(current_loc);
        for( weighted_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            int value = *it;
            int index = it.index();
            int slabels = get_slabels(beam, value);
            float p = base_->probability_obs(obs, current_loc, slabels, last_action);
            int k_obs = std::min(kappa_t::kappa(p), max_kappa_);
            weight_increases.push_back(std::make_pair(index, k_obs));
        }
#ifdef DEBUG
        std::cout << "  increases:";
        for( int i = 0; i < int(weight_increases.size()); ++i ) {
            std::pair<int, int> &p = weight_increases[i];
            int valuation = beam[p.first].first;
            int weight = beam[p.first].second;
#if 0
            std::cout << " {" << p.first << ":" << valuation << ":";
            std::map<dai::Var, size_t> state = dai::calcState(beam.varset(), valuation);
            for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it )
                std::cout << it->first << "=" << it->second << ",";
            std::cout << "weight=" << weight << ",amount=" << p.second << "},";
#else
            std::cout << " " << p.second;
#endif
        }
        std::cout << std::endl;
#endif

        assert(!beam.empty());
        for( int i = 0; i < int(weight_increases.size()); ++i )
            weighted_csp_.domain(current_loc)->increase_weight(weight_increases[i], max_kappa_);
        assert(weighted_csp_.worklist().empty());
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
    bool update_factors_gm(int last_action, int obs) {
        assert(!use_ac3_);
        int current_loc = loc_history_.back();
#ifdef DEBUG
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  factor: ";
        inference_.print_factor(std::cout, current_loc, factors_, "factors");
        std::cout << std::endl;
#endif
        assert(current_loc < int(factors_.size()));
        dai::Factor &factor = factors_[current_loc];
        for( int value = 0; value < int(factor.nrStates()); ++value ) {
            int slabels = get_slabels(current_loc, factor.vars(), value);
            factor.set(value, factor[value] * base_->probability_obs(obs, current_loc, slabels, last_action));
        }
        factor.normalize();
        indices_for_updated_factors_.push_back(current_loc);
#ifdef DEBUG
        std::cout << "factor after update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  factor: ";
        inference_.print_factor(std::cout, current_loc, factors_, "factors");
        std::cout << std::endl;
#endif
        return true;
    }
    bool update_factors(int last_action, int obs) {
        return use_ac3_ ? update_factors_ac3(last_action, obs) : update_factors_gm(last_action, obs);
    }

    void calculate_marginals(mpi_slam_t *mpi, int wid, bool print_marginals = false) const {
#ifndef USE_MPI
        inference_.calculate_marginals(cache_t::variables(),
                                       indices_for_updated_factors_,
                                       factors_,
                                       marginals_,
                                       Inference::edbp_t::edbp_factor_index,
                                       print_marginals);
#else
        assert(mpi != 0);
        assert((wid > 0) && (wid < mpi->nworkers_));
        mpi->calculate_marginals(factors_, indices_for_updated_factors_, wid);
        assert(indices_for_updated_factors_.empty());
#endif
    }

    void update_marginals(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            if( use_ac3_ ) {
                const weighted_varset_beam_t &beam = *weighted_csp_.domain(loc);
                for( int label = 0; label < base_->nlabels_; ++label )
                    marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * beam.marginal(label));
            } else {
                dai::Factor marginal = marginals_[loc].marginal(dai::Var(loc, 2));
                assert(base_->nlabels_ == int(marginal.nrStates()));
                for( int label = 0; label < base_->nlabels_; ++label )
                    marginals_on_vars[loc].set(label, marginals_on_vars[loc][label] + weight * marginal[label]);
            }
        }
        int current_loc = loc_history_.back();
        marginals_on_vars[base_->nloc_].set(current_loc, marginals_on_vars[base_->nloc_][current_loc] + weight);
    }

    int value_for(int /*var*/) const { return -1; }

    void mpi_update_marginals(mpi_slam_t *mpi, int wid) {
#ifdef USE_MPI
        mpi->read_marginals_from_worker(marginals_, wid);
#endif
    }

    void print(std::ostream &os) const {
        if( loc_history_.empty() )
            os << "loc=<empty history>";
        else
            os << "loc=" << loc_history_.back();
    }
};

// Particle for the motion model RBPF filter
struct motion_model_rbpf_particle_t : public rbpf_particle_t {
    motion_model_rbpf_particle_t(bool use_ac3, int max_kappa) : rbpf_particle_t(use_ac3, max_kappa) { }
    motion_model_rbpf_particle_t(const std::multimap<std::string, std::string> &parameters) : rbpf_particle_t(parameters) { }
    motion_model_rbpf_particle_t(const motion_model_rbpf_particle_t &p) : rbpf_particle_t(p) { }
    motion_model_rbpf_particle_t(motion_model_rbpf_particle_t &&p) : rbpf_particle_t(std::move(p)) { }
    ~motion_model_rbpf_particle_t() { }

    const motion_model_rbpf_particle_t& operator=(const motion_model_rbpf_particle_t &p) {
        *static_cast<rbpf_particle_t*>(this) = p;
        return *this;
    }
    bool operator==(const motion_model_rbpf_particle_t &p) const {
        return *static_cast<const rbpf_particle_t*>(this) == p;
    }

    static std::string type() {
        return std::string("mm_rbpf_sir");
    }

    virtual bool sample_from_pi(rbpf_particle_t &np,
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
            bool status = np.update_factors(last_action, obs);
            if( !use_ac3_ && status ) np.calculate_marginals(mpi, wid, false);
            return status;
        }
        return true;
    }

    virtual float importance_weight(const rbpf_particle_t &np, int last_action, int obs) const {
        assert(indices_for_updated_factors_.empty());
        int np_current_loc = np.loc_history_.back();
        float weight = 0;
        if( use_ac3_ ) {
            const weighted_varset_beam_t &beam = *weighted_csp_.domain(np_current_loc);
            for( weighted_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
                int slabels = get_slabels(beam, *it);
                weight += beam.probability(*it, it.weight()) * base_->probability_obs(obs, np_current_loc, slabels, last_action);
            }
        } else {
            const dai::Factor &marginal = marginals_[np_current_loc];
            for( int value = 0; value < int(marginal.nrStates()); ++value ) {
                int slabels = get_slabels(np_current_loc, marginal.vars(), value);
                weight += marginal[value] * base_->probability_obs(obs, np_current_loc, slabels, last_action);
            }
        }
        return weight;
    }

    motion_model_rbpf_particle_t* initial_sampling(mpi_slam_t *mpi, int wid) {
        motion_model_rbpf_particle_t *p = new motion_model_rbpf_particle_t(use_ac3_, max_kappa_);
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

// Particle for the optimal RBPF filter (verified: 09/12/2015)
struct optimal_rbpf_particle_t : public rbpf_particle_t {
    mutable std::vector<float> cdf_;

    optimal_rbpf_particle_t(bool use_ac3, int max_kappa) : rbpf_particle_t(use_ac3, max_kappa) { }
    optimal_rbpf_particle_t(const std::multimap<std::string, std::string> &parameters) : rbpf_particle_t(parameters) { }
    optimal_rbpf_particle_t(const optimal_rbpf_particle_t &p) : rbpf_particle_t(p) { }
    optimal_rbpf_particle_t(optimal_rbpf_particle_t &&p) : rbpf_particle_t(std::move(p)) { }
    ~optimal_rbpf_particle_t() { }

    const optimal_rbpf_particle_t& operator=(const optimal_rbpf_particle_t &p) {
        *static_cast<rbpf_particle_t*>(this) = p;
        return *this;
    }
    bool operator==(const optimal_rbpf_particle_t &p) const {
        return *static_cast<const rbpf_particle_t*>(this) == p;
    }

    static std::string type() {
        return std::string("opt_rbpf_sir");
    }

    void calculate_cdf(int last_action, int obs, std::vector<float> &cdf) const {
        // make sure there is no pending inference on factor model
        assert(indices_for_updated_factors_.empty());

        cdf.clear();
        cdf.reserve(base_->nloc_);

        // P(nloc | loc, action, obs) = alpha * P(nloc, obs | loc, action)
        //                            = alpha * P(obs | nloc, loc, action) * P(nloc | loc, action)
        //                            = alpha * P(obs | nloc, action) * P(nloc | loc, action)

        float previous = 0;
        int current_loc = loc_history_.back();
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            float p = 0;
            if( use_ac3_ ) {
                const weighted_varset_beam_t &beam = *weighted_csp_.domain(nloc);
                for( weighted_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
                    int slabels = get_slabels(beam, *it);
                    p += beam.probability(*it, it.weight()) * base_->probability_obs(obs, nloc, slabels, last_action);
                }
            } else {
                const dai::Factor &marginal = marginals_[nloc];
                for( int value = 0; value < int(marginal.nrStates()); ++value ) {
                    int slabels = get_slabels(nloc, marginal.vars(), value);
                    p += marginal[value] * base_->probability_obs(obs, nloc, slabels, last_action);
                }
            }
            cdf.push_back(previous + base_->probability_tr_loc(last_action, current_loc, nloc) * p);
            previous = cdf.back();
        }

        // normalize (i.e. calculate alpha)
        assert(cdf.back() > 0);
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            cdf[nloc] /= cdf.back();
        }
    }

    virtual bool sample_from_pi(rbpf_particle_t &np,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                mpi_slam_t *mpi,
                                int wid) const {
#ifdef DEBUG
        assert(np == *this);
#endif
        calculate_cdf(last_action, obs, cdf_);
        int next_loc = Utils::sample_from_distribution(base_->nloc_, &cdf_[0]);
        np.loc_history_.push_back(next_loc);
        if( !history_container.contains(np.loc_history_) ) {
            // this is a new loc history, perform update
            bool status = np.update_factors(last_action, obs);
            if( !use_ac3_ && status ) np.calculate_marginals(mpi, wid, false);
            return status;
        }
        return true;
    }

    virtual float importance_weight(const rbpf_particle_t &, int last_action, int obs) const {
        int current_loc = loc_history_.back();
        float weight = 0;
        for( int nloc = 0; nloc < base_->nloc_; ++nloc ) {
            float p = 0;
            if( use_ac3_ ) {
                const weighted_varset_beam_t &beam = *weighted_csp_.domain(nloc);
                for( weighted_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
                    int slabels = get_slabels(beam, *it);
                    p += beam.probability(*it, it.weight()) * base_->probability_obs(obs, nloc, slabels, last_action);
                }
            } else {
                const dai::Factor &marginal = marginals_[nloc];
                for( int value = 0; value < int(marginal.nrStates()); ++value ) {
                    int slabels = get_slabels(nloc, marginal.vars(), value);
                    p += marginal[value] * base_->probability_obs(obs, nloc, slabels, last_action);
                }
            }
            weight += base_->probability_tr_loc(last_action, current_loc, nloc) * p;
        }
        return weight;
    }

    optimal_rbpf_particle_t* initial_sampling(mpi_slam_t *mpi, int wid) {
        optimal_rbpf_particle_t *p = new optimal_rbpf_particle_t(use_ac3_, max_kappa_);
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

}; // namespace OreSLAM

#undef DEBUG

#endif

