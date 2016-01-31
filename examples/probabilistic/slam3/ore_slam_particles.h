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
#include <strings.h>
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
#include "iterated_weighted_arc_consistency.h"

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
                            unsigned *bitmask = new unsigned[size]();
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
                                    bitmask[index] |= 1 << offset;
                                    ++ncompatible;
                                }
                            }
                            assert((ncompatible > 0) && (ncompatible < int(varsets_[var_x].nrStates())));
                            assert(val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x < int(compatible_values_.size()));
                            compatible_values_[val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x] = bitmask;
                            cache.insert(std::make_pair(cache_key, bitmask));
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
            unsigned *bitmask = compatible_values_[i];
            if( (bitmask != 0) && (erased.find(bitmask) == erased.end()) ) {
                delete[] bitmask;
                erased.insert(bitmask);
            }
        }
        SLAM::cache_t::finalize();
    }

    static const unsigned* compatible_values(int val_y, int var_y, int var_x) {
        return compatible_values_[val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x];
    }
};

class kappa_varset_beam_t : public weighted_var_beam_t {
  protected:
    const int loc_;
    const dai::Var &var_;
    const dai::VarSet &varset_;
    unsigned mask_;
    float normalization_constant_;

  public:
    kappa_varset_beam_t(int loc, const dai::Var &var, const dai::VarSet &varset)
      : weighted_var_beam_t(1, varset.nrStates()), loc_(loc), var_(var), varset_(varset), mask_(0), normalization_constant_(1) {
        mask_ = unsigned(-1);
        for( dai::State state(varset_); state.valid(); state++ ) {
            if( state(var_) == 1 )
                mask_ = mask_ & dai::calcLinearState(varset_, state);
        }
#ifdef DEBUG
        std::cout << "kappa_varset_beam_t: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    kappa_varset_beam_t(const kappa_varset_beam_t &beam)
      : weighted_var_beam_t(beam),
        loc_(beam.loc_),
        var_(beam.var_),
        varset_(beam.varset_),
        mask_(beam.mask_),
        normalization_constant_(beam.normalization_constant_) {
#ifdef DEBUG
        std::cout << "kappa_varset_beam_t: copy-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    kappa_varset_beam_t(kappa_varset_beam_t &&beam)
      : weighted_var_beam_t(std::move(beam)),
        loc_(beam.loc_),
        var_(beam.var_),
        varset_(std::move(beam.varset_)),
        mask_(beam.mask_),
        normalization_constant_(beam.normalization_constant_) {
#ifdef DEBUG
        std::cout << "kappa_varset_beam_t: move-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << std::endl;
#endif
    }
    ~kappa_varset_beam_t() {
    }

    bool operator==(const kappa_varset_beam_t &beam) const {
        return (loc_ == beam.loc_) && (*static_cast<const weighted_var_beam_t*>(this) == *static_cast<const weighted_var_beam_t*>(&beam));
    }

    int loc() const { return loc_; }
    const dai::VarSet& varset() const { return varset_; }

    void set_kappa(int index, int kappa) {
        weighted_var_beam_t::set_weight(index, kappa);
    }
    void increase_kappa(const std::pair<int, int> &p) {
        weighted_var_beam_t::increase_weight(p);
    }

    float probability(unsigned valuation, int kappa) const {
        return kappa == std::numeric_limits<int>::max() ? 0 : normalization_constant_ * kappa_t::power(kappa);
    }
    float probability(int valuation) const {
        int kappa = weight(valuation);
        return kappa == -1 ? 0 : probability(valuation, kappa);
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

class kappa_arc_consistency_t : public CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t> {
    static CSP::constraint_digraph_t cg_;

    int iterated_level_;
    bool inverse_check_;

    mutable int level_;
    //mutable const std::map<dai::Var, size_t> *state_x_;
    mutable unsigned bitmask_[16];

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
        const unsigned *bitmask = cache_t::compatible_values(val_y, var_y, var_x);
        int index = val_x / 32, offset = val_x % 32;
        return (bitmask[index] & (1 << offset)) != 0;
#endif
    }

    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y, int level) const {
        level_ = level;
        bzero(bitmask_, 64); // 16 unsigned ints = 64 bytes
    }
    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y, int val_y, int kappa) const {
        if( kappa <= level_ ) {
            const unsigned *bitmask = cache_t::compatible_values(val_y, var_y, var_x);
            for( int i = 0; i < 16; ++i ) bitmask_[i] |= bitmask[i];
        }
    }
    virtual bool arc_reduce_inverse_check(int val_x, int kappa) const {
        int index = val_x / 32, offset = val_x % 32;
        return ((bitmask_[index] >> offset) & 0x1) == 1;
    }

  public:
    kappa_arc_consistency_t()
      : CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t>(cg_),
        iterated_level_(0),
        inverse_check_(false) {
    }
    kappa_arc_consistency_t(const kappa_arc_consistency_t &ac)
      : CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t>(ac.cg_),
        iterated_level_(ac.iterated_level_),
        inverse_check_(ac.inverse_check_) {
    }
    kappa_arc_consistency_t(kappa_arc_consistency_t &&ac)
      : CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t>(std::move(ac)),
        iterated_level_(ac.iterated_level_),
        inverse_check_(ac.inverse_check_) {
    }
    virtual ~kappa_arc_consistency_t() { }

    const kappa_arc_consistency_t& operator=(const kappa_arc_consistency_t &ac) {
        nvars_ = ac.nvars_;
        iterated_level_ = ac.iterated_level_;
        inverse_check_ = ac.inverse_check_;
        assert(domain_.size() == ac.domain_.size());
        for( int loc = 0; loc < int(ac.domain_.size()); ++loc )
            set_domain(loc, new kappa_varset_beam_t(*ac.domain(loc)));
        return *this;
    }
    bool operator==(const kappa_arc_consistency_t &ac) const {
        if( (nvars_ == ac.nvars_) && (iterated_level_ == ac.iterated_level_) && (inverse_check_ == ac.inverse_check_) && (domain_.size() == ac.domain_.size()) ) {
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

    void set_iterated_level(int iterated_level) {
        iterated_level_ = iterated_level;
    }
    void set_inverse_check(bool inverse_check) {
        inverse_check_ = inverse_check;
    }

    bool kappa_ac3(std::vector<int> &revised_vars) {
        bool something_removed = CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t>::iterated_weighted_ac3(iterated_level_, revised_vars, true, inverse_check_);
        return something_removed;
    }

    void calculate_normalization_constant(int loc) {
        domain_[loc]->calculate_normalization_constant();
    }
    void calculate_normalization_constant(const std::vector<int> &revised_vars) {
        for( int i = 0; i < int(revised_vars.size()); ++i )
            calculate_normalization_constant(revised_vars[i]);
    }
    void calculate_normalization_constant() {
        for( int loc = 0; loc < nvars_; ++loc )
            calculate_normalization_constant(loc);
    }

    void normalize_kappas(int loc) {
        domain_[loc]->normalize();
    }
    void normalize_kappas(const std::vector<int> &revised_vars) {
        for( int i = 0; i < int(revised_vars.size()); ++i )
            normalize_kappas(revised_vars[i]);
    }
    void normalize_kappas() {
        for( int loc = 0; loc < nvars_; ++loc )
            normalize_kappas(loc);
    }

    bool normalized_kappas(int loc) {
        return domain_[loc]->normalized();
    }
    bool normalized_kappas() {
        for( int i = 0; i < int(domain_.size()); ++i ) {
            if( !normalized_kappas(i) )
                return false;
        }
        return true;
    }

    void print_factor(std::ostream &os, int loc) const {
        const kappa_varset_beam_t &beam = *domain_[loc];
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
    bool lazy_ac_;
    bool simple_ac_;
    int iterated_level_;
    bool inverse_check_;
    kappa_arc_consistency_t kappa_csp_;

    // factors
    std::vector<dai::Factor> factors_;

    // computation of marginals in factor model
    mutable std::vector<int> indices_for_updated_factors_;
    mutable std::vector<dai::Factor> marginals_;
    Inference::inference_t inference_;

    rbpf_particle_t(bool use_ac3, bool lazy_ac, bool simple_ac, int iterated_level, bool inverse_check, const Inference::inference_t *i)
      : use_ac3_(use_ac3), lazy_ac_(lazy_ac), simple_ac_(simple_ac), iterated_level_(iterated_level), inverse_check_(inverse_check)  {
        assert(base_ != 0);
        assert(base_->nlabels_ == 2);

        if( use_ac3_ ) {
            kappa_csp_.set_iterated_level(iterated_level_);
            kappa_csp_.set_inverse_check(inverse_check_);
        } else {
            kappa_csp_.delete_domains_and_clear();
        }

        if( !use_ac3_ || !lazy_ac_ ) {
            factors_ = std::vector<dai::Factor>(base_->nloc_);
            marginals_ = std::vector<dai::Factor>(base_->nloc_);
        }

        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            if( use_ac3_ ) {
                kappa_csp_.set_domain(loc, new kappa_varset_beam_t(loc, cache_t::variable(loc), cache_t::varset(loc)));
            }
            if( !use_ac3_ || !lazy_ac_ ) {
                factors_[loc] = dai::Factor(cache_t::varset(loc));
                marginals_[loc] = dai::Factor(cache_t::varset(loc));
            }
        }

        if( !use_ac3_ && (i != 0) ) inference_ = *i;
    }
    rbpf_particle_t(const std::multimap<std::string, std::string> &parameters)
      : rbpf_particle_t(false, false, false, 0, false, 0) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("inference");
        if( it != parameters.end() ) {
            inference_.set_inference_algorithm(it->second, "BEL", false);
            if( inference_.algorithm() == "iterated-ac3" ) {
                const dai::PropertySet &options = inference_.options();
                use_ac3_ = true;
                if( options.hasKey("level") )
                    iterated_level_ = int(options.getStringAs<size_t>("level"));
                if( options.hasKey("inverse-check") )
                    inverse_check_ = options.getStringAs<bool>("inverse-check");
                if( options.hasKey("lazy") )
                    lazy_ac_ = options.getStringAs<bool>("lazy");
                if( options.hasKey("simple") )
                    simple_ac_ = options.getStringAs<bool>("simple");
                kappa_csp_.set_iterated_level(iterated_level_);
                kappa_csp_.set_inverse_check(inverse_check_);
            } else {
                use_ac3_ = false;
                inference_.create_and_initialize_algorithm(factors_);
            }
        }
        it = parameters.find("edbp-max-iter");
        if( it != parameters.end() )
            inference_.edbp_max_iter_ = strtoul(it->second.c_str(), 0, 0);
    }
    rbpf_particle_t(const rbpf_particle_t &p) {
        if( !p.use_ac3_ ) kappa_csp_.delete_domains_and_clear();
        *this = p;
    }
    rbpf_particle_t(rbpf_particle_t &&p)
      : loc_history_(std::move(p.loc_history_)),
        use_ac3_(p.use_ac3_),
        lazy_ac_(p.lazy_ac_),
        simple_ac_(p.simple_ac_),
        iterated_level_(p.iterated_level_),
        inverse_check_(p.inverse_check_),
        kappa_csp_(std::move(p.kappa_csp_)),
        factors_(std::move(p.factors_)),
        indices_for_updated_factors_(std::move(p.indices_for_updated_factors_)),
        marginals_(std::move(p.marginals_)),
        inference_(std::move(p.inference_)) {
    }
    virtual ~rbpf_particle_t() {
        kappa_csp_.delete_domains_and_clear();
        if( !use_ac3_) inference_.destroy_inference_algorithm();
    }

    const rbpf_particle_t& operator=(const rbpf_particle_t &p) {
        loc_history_ = p.loc_history_;
        use_ac3_ = p.use_ac3_;
        lazy_ac_ = p.lazy_ac_;
        simple_ac_ = p.simple_ac_;
        iterated_level_ = p.iterated_level_;
        inverse_check_ = p.inverse_check_;
        kappa_csp_ = p.kappa_csp_;
        factors_ = p.factors_;
        indices_for_updated_factors_ = p.indices_for_updated_factors_;
        marginals_ = p.marginals_;
        inference_ = p.inference_;
        return *this;
    }

    bool operator==(const rbpf_particle_t &p) const {
        return (loc_history_ == p.loc_history_) &&
               (use_ac3_ == p.use_ac3_) &&
               (lazy_ac_ == p.lazy_ac_) &&
               (simple_ac_ == p.simple_ac_) &&
               (iterated_level_ == p.iterated_level_) &&
               (inverse_check_ == p.inverse_check_) &&
               (kappa_csp_ == p.kappa_csp_) &&
               (factors_ == p.factors_) &&
               (indices_for_updated_factors_ == p.indices_for_updated_factors_) &&
               (marginals_ == p.marginals_) &&
               (inference_ == p.inference_);
    }

    void reset_csp() {
        assert(use_ac3_);
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            kappa_csp_.domain(loc)->set_initial_configuration();
            kappa_csp_.calculate_normalization_constant(loc);
        }
        assert(kappa_csp_.normalized_kappas());
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

        if( !use_ac3_ ) {
            indices_for_updated_factors_.clear();
            indices_for_updated_factors_.reserve(base_->nloc_);
        }

        // set initial history and reset factors for locations
        loc_history_.push_back(base_->initial_loc_);
        if( !use_ac3_ || !lazy_ac_ ) {
            for( int loc = 0; loc < base_->nloc_; ++loc ) {
                dai::Factor &factor = factors_[loc];
                float p = 1.0 / (1 << factor.vars().size());
                for( int i = 0; i < (1 << factor.vars().size()); ++i )
                    factor.set(i, p);
                if( !use_ac3_ ) indices_for_updated_factors_.push_back(loc);
            }
        }

        if( use_ac3_ ) {
            reset_csp();
        } else {
            calculate_marginals(mpi, wid, false);
        }
    }

    static int var_offset(int loc, int var_id) { return base_->var_offset(loc, var_id); }
    int get_slabels(const kappa_varset_beam_t &beam, int value) const {
        return cache_t::get_slabels(beam.loc(), beam.varset(), value, var_offset);
    }
    int get_slabels(int loc, const dai::VarSet &varset, int value) const {
        return cache_t::get_slabels(loc, varset, value, var_offset);
    }

    bool update_factors_ac3(int last_action, int obs) {
        assert(use_ac3_);
        int current_loc = loc_history_.back();
        assert(current_loc < int(kappa_csp_.nvars()));
        assert(lazy_ac_ || (current_loc < int(factors_.size())));

        // first update factors in standard manner
        if( !lazy_ac_ ) {
            update_factors_gm(last_action, obs);
        }

#ifdef DEBUG
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  kappa-csp: ";
        kappa_csp_.print_factor(std::cout, current_loc);
        std::cout << std::endl;
#endif

        // second, increase kappas (if lazy) or translate factors into kappa tables,
        // keeping track of changes
        assert(kappa_csp_.worklist().empty());
        if( lazy_ac_ ) {
            std::vector<std::pair<int, int> > kappa_increases;
            kappa_varset_beam_t &beam = *kappa_csp_.domain(current_loc);
            for( kappa_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
                int value = *it;
                int index = it.index();
                int slabels = get_slabels(beam, value);
                float p = base_->probability_obs(obs, current_loc, slabels, last_action);
                int k_obs = kappa_t::kappa(p);
                assert(k_obs >= 0);
                if( k_obs > 0 ) kappa_increases.push_back(std::make_pair(index, k_obs));
            }

#ifdef DEBUG
            std::cout << "  increases:";
            for( int i = 0; i < int(kappa_increases.size()); ++i ) {
                std::pair<int, int> &p = kappa_increases[i];
                //int valuation = beam[p.first].first;
                //int kappa = beam[p.first].second;
                std::cout << " " << p.second;
            }
            std::cout << std::endl;
#endif

            // do increases
            for( int i = 0; i < int(kappa_increases.size()); ++i )
                beam.increase_kappa(kappa_increases[i]);

            if( !kappa_increases.empty() ) {
                kappa_csp_.normalize_kappas(current_loc);
                kappa_csp_.add_to_worklist(current_loc);
            }
        } else {
            for( int loc = 0; loc < int(factors_.size()); ++loc ) {
                const dai::Factor &factor = factors_[loc];
                kappa_varset_beam_t &beam = *kappa_csp_.domain(loc);
                bool change = false;
                for( kappa_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
                    int value = *it;
                    int kappa = it.weight();
                    float p = factor[value];
                    int new_kappa = kappa_t::kappa(p);
                    if( new_kappa != kappa ) {
                        change = true;
                        beam.set_kappa(it.index(), new_kappa);
                    }
                }
                if( change ) {
                    kappa_csp_.normalize_kappas(loc);
                    if( !simple_ac_ || (simple_ac_ && (loc == current_loc)) )
                        kappa_csp_.add_to_worklist(loc);
                }
            }
        }

        // propagate change in kappa values to other tables (factors)
        if( !kappa_csp_.worklist().empty() ) {
            std::vector<int> revised_vars;
            kappa_csp_.kappa_ac3(revised_vars);
        }
        kappa_csp_.normalize_kappas();
        kappa_csp_.calculate_normalization_constant();

#ifdef DEBUG
        std::cout << "factor after update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  kappa-csp: ";
        kappa_csp_.print_factor(std::cout, current_loc);
        std::cout << std::endl;
#endif

        return kappa_csp_.is_consistent(0);
    }
    bool update_factors_gm(int last_action, int obs) {
        int current_loc = loc_history_.back();
        assert(current_loc < int(factors_.size()));

#ifdef DEBUG
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  factor: ";
        inference_.print_factor(std::cout, current_loc, factors_, "factors");
        std::cout << std::endl;
#endif

        dai::Factor &factor = factors_[current_loc];
        for( int value = 0; value < int(factor.nrStates()); ++value ) {
            int slabels = get_slabels(current_loc, factor.vars(), value);
            factor.set(value, factor[value] * base_->probability_obs(obs, current_loc, slabels, last_action));
        }
        factor.normalize();
        if( !use_ac3_ ) indices_for_updated_factors_.push_back(current_loc);

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
#endif
        assert(indices_for_updated_factors_.empty());
    }

    void update_marginals(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            if( use_ac3_ ) {
                const kappa_varset_beam_t &beam = *kappa_csp_.domain(loc);
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
    motion_model_rbpf_particle_t(bool use_ac3, bool lazy_ac, bool simple_ac, int iterated_level, bool inverse_check, const Inference::inference_t *i)
      : rbpf_particle_t(use_ac3, lazy_ac, simple_ac, iterated_level, inverse_check, i) { }
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
            const kappa_varset_beam_t &beam = *kappa_csp_.domain(np_current_loc);
            for( kappa_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
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
        motion_model_rbpf_particle_t *p = new motion_model_rbpf_particle_t(use_ac3_, lazy_ac_, simple_ac_, iterated_level_, inverse_check_, &inference_);
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

// Particle for the optimal RBPF filter (verified: 09/12/2015)
struct optimal_rbpf_particle_t : public rbpf_particle_t {
    mutable std::vector<float> cdf_;

    optimal_rbpf_particle_t(bool use_ac3, bool lazy_ac, bool simple_ac, int iterated_level, bool inverse_check, const Inference::inference_t *i)
      : rbpf_particle_t(use_ac3, lazy_ac, simple_ac, iterated_level, inverse_check, i) { }
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
                const kappa_varset_beam_t &beam = *kappa_csp_.domain(nloc);
                for( kappa_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
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
                const kappa_varset_beam_t &beam = *kappa_csp_.domain(nloc);
                for( kappa_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
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
        optimal_rbpf_particle_t *p = new optimal_rbpf_particle_t(use_ac3_, lazy_ac_, simple_ac_, iterated_level_, inverse_check_, &inference_);
        p->initial_sampling_in_place(mpi, wid);
        return p;
    }
};

}; // namespace OreSLAM

#undef DEBUG

#endif

