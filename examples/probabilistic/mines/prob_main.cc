/*
 *  Copyright (C) 2015-2016 Universidad Simon Bolivar
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

#include <cassert>
#include <cstdlib>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <math.h>

#include <dai/alldai.h>
#include "../slam3/inference.h"
#include "../slam3/edbp.h"
#include "../slam3/utils.h"

#include "kappa.h"
#include "libdai_cache.h"
#include "weighted_var_beam.h"
#include "iterated_weighted_arc_consistency.h"

using namespace std;

// static members for inference algorithms
string Inference::inference_t::edbp_factors_fn_;
string Inference::inference_t::edbp_evid_fn_;
string Inference::inference_t::edbp_output_fn_;
vector<vector<int> > Inference::edbp_t::edbp_factor_indices_;

// static members for kappa handling
float kappa_t::epsilon_ = 0;
vector<float> kappa_t::powers_;

// static member for dai cache
int dai::cache_t::num_locs_ = 0;
vector<dai::Var> dai::cache_t::variables_;
vector<dai::VarSet> dai::cache_t::varsets_;
vector<vector<map<dai::Var, size_t>*> > dai::cache_t::state_cache_;

class cache_t : public dai::cache_t {
  protected:
    static vector<unsigned*> compatible_values_;

    static void compute_cache_for_compatible_values(int nrows, int ncols) {
        float start_time = Utils::read_time_in_seconds();

        // for each location (var_y) and value for it, for each adjacent loc in constraint graph,
        // determine compatible values
        map<string, unsigned*> cache;
        unsigned cache_hits = 0;
        compatible_values_ = vector<unsigned*>(num_locs_ * num_locs_ * 512, static_cast<unsigned*>(0));
        for( int var_y = 0; var_y < num_locs_; ++var_y ) {
            int row = var_y / ncols, col = var_y % ncols;
            for( int val_y = 0; val_y < int(varsets_[var_y].nrStates()); ++val_y ) {
                const map<dai::Var, size_t> &state_y = *state_cache_[var_y][val_y];
                for( int dc = -2; dc < 3; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= ncols) ) continue;
                    for( int dr = -2; dr < 3; ++dr ) {
                        int nr = row + dr;
                        if( (nr < 0) || (nr >= nrows) ) continue;
                        int var_x = nr * ncols + nc;
                        if( var_x == var_y ) continue;

                        // check whether we have already calculated this entry
                        string cache_key;
                        cache_key += to_string(varsets_[var_y].size()) + " ";
                        cache_key += to_string(varsets_[var_x].size()) + " ";
                        cache_key += to_string(dr) + " ";
                        cache_key += to_string(dc) + " ";
                        cache_key += to_string(val_y) + " ";
                        cache_key += to_string(row == 0);
                        cache_key += to_string(row == nrows - 1);
                        cache_key += to_string(col == 0);
                        cache_key += to_string(col == ncols - 1);
                        cache_key += to_string(nr == 0);
                        cache_key += to_string(nr == nrows - 1);
                        cache_key += to_string(nc == 0);
                        cache_key += to_string(nc == ncols - 1);
                        map<string, unsigned*>::const_iterator cache_it = cache.find(cache_key);
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
                                const map<dai::Var, size_t> &state_x = *state_cache_[var_x][val_x];
                                for( map<dai::Var, size_t>::const_iterator it = state_x.begin(); it != state_x.end(); ++it ) {
                                    map<dai::Var, size_t>::const_iterator jt = state_y.find(it->first);
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
                            cache.insert(make_pair(cache_key, values));
                        }
                    }
                }
            }
        }
        cout << "# cache-for-compatible-values:"
             << " size=" << cache.size()
             << ", hits=" << cache_hits
             << ", time=" << Utils::read_time_in_seconds() - start_time
             << endl;
    }

  public:
    cache_t() { }
    ~cache_t() { }

    static void initialize(int nrows, int ncols) {
        num_locs_ = nrows * ncols;
        dai::cache_t::compute_basic_elements(nrows, ncols);
        dai::cache_t::compute_cache_for_states(nrows, ncols);
        compute_cache_for_compatible_values(nrows, ncols);
    }
    static void finalize() {
        set<unsigned*> erased;
        for( int i = 0; i < int(compatible_values_.size()); ++i ) {
            unsigned *values = compatible_values_[i];
            if( (values != 0) && (erased.find(values) == erased.end()) ) {
                delete[] values;
                erased.insert(values);
            }
        }
        dai::cache_t::finalize();
    }

    static const unsigned* compatible_values(int val_y, int var_y, int var_x) {
        return compatible_values_[val_y * num_locs_ * num_locs_ + var_y * num_locs_ + var_x];
    }
};

// static members for cache
vector<unsigned*> cache_t::compatible_values_;

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
        cout << "kappa_varset_beam_t: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << endl;
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
        cout << "kappa_varset_beam_t: copy-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << endl;
#endif
    }
    kappa_varset_beam_t(kappa_varset_beam_t &&beam)
      : weighted_var_beam_t(move(beam)),
        loc_(beam.loc_),
        var_(beam.var_),
        varset_(move(beam.varset_)),
        mask_(beam.mask_),
        normalization_constant_(beam.normalization_constant_) {
#ifdef DEBUG
        cout << "kappa_varset_beam_t: move-constructor: loc=" << loc_ << ", var=" << var_ << ", varset=" << varset_ << ", mask=" << mask_ << endl;
#endif
    }
    ~kappa_varset_beam_t() {
    }

    bool operator==(const kappa_varset_beam_t &beam) const {
        return (loc_ == beam.loc_) && (*static_cast<const weighted_var_beam_t*>(this) == *static_cast<const weighted_var_beam_t*>(&beam));
    }

    int loc() const { return loc_; }
    const dai::VarSet& varset() const { return varset_; }

    void increase_kappa(const pair<int, int> &p) {
        weighted_var_beam_t::increase_weight(p);
    }

    float probability(unsigned valuation, int kappa) const {
        return kappa == numeric_limits<int>::max() ? 0 : normalization_constant_ * kappa_t::power(kappa);
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
    void erase_ordered_indices(const vector<int> &ordered_indices) {
        weighted_var_beam_t::erase_ordered_indices(ordered_indices);
    }
};

class kappa_arc_consistency_t : public CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t> {
    static CSP::constraint_digraph_t cg_;

    int iterated_level_;
    bool inverse_check_;

    mutable int level_;
    //mutable const map<dai::Var, size_t> *state_x_;
    mutable vector<unsigned*> bitmask_;
    //mutable int bitmask_size_;

    static void construct_constraint_graph(int nrows, int ncols) {
        assert(0);
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
        cout << "# cg: #vars=" << cg_.nvars() << ", #edges=" << cg_.nedges() << endl;
    }

    virtual void arc_reduce_preprocessing_1(int var_x, int val_x) const {
        //state_x_ = &cache_t::state(var_x, val_x);
    }
    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const {
#if 0
        const map<dai::Var, size_t> &state_y = cache_t::state(var_y, val_y);
        for( map<dai::Var, size_t>::const_iterator it = state_x_->begin(); it != state_x_->end(); ++it ) {
            map<dai::Var, size_t>::const_iterator jt = state_y.find(it->first);
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

    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y, int level) const {
        assert(1 + level <= int(bitmask_.size()));
        level_ = level;
        //bitmask_size_ = cache_t::varset(var_x).nrStates() >> 5;
        //bitmask_size_ = bitmask_size_ == 0 ? 1 : bitmask_size_;
        for( int i = 0; i <= level_; ++i )
            bzero(bitmask_[i], 64);
    }
    virtual void arc_reduce_inverse_check_preprocessing(int var_x, int var_y, int val_y, int kappa) const {
        if( kappa <= level_ ) {
            assert(1 + kappa <= int(bitmask_.size()));
            const unsigned *bitmask = cache_t::compatible_values(val_y, var_y, var_x);
            for( int i = 0; i < 16; ++i ) bitmask_[kappa][i] |= bitmask[i];
        }
    }
    virtual bool arc_reduce_inverse_check(int val_x, int kappa) const {
        if( kappa <= level_ ) {
            assert(1 + level_ - kappa <= int(bitmask_.size()));
            int index = val_x / 32, offset = val_x % 32;
            //assert(index < bitmask_size_);
            for( int i = 0; i <= level_ - kappa; ++i ) {
                if( ((bitmask_[i][index] >> offset) & 0x1) == 1 )
                    return true;
            }
            return false;
        } else {
            return true;
        }
    }

  public:
    kappa_arc_consistency_t()
      : CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t>(cg_),
        iterated_level_(0),
        inverse_check_(false) {
        set_iterated_level(iterated_level_);
    }
    kappa_arc_consistency_t(const kappa_arc_consistency_t &ac)
      : CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t>(ac.cg_),
        iterated_level_(ac.iterated_level_),
        inverse_check_(ac.inverse_check_) {
        set_iterated_level(iterated_level_);
    }
    kappa_arc_consistency_t(kappa_arc_consistency_t &&ac)
      : CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t>(move(ac)),
        iterated_level_(ac.iterated_level_),
        inverse_check_(ac.inverse_check_),
        bitmask_(move(ac.bitmask_)) {
        ac.bitmask_.clear();
    }
    virtual ~kappa_arc_consistency_t() {
        clean_bitmask();
    }

    const kappa_arc_consistency_t& operator=(const kappa_arc_consistency_t &ac) {
        nvars_ = ac.nvars_;
        set_iterated_level(ac.iterated_level_);
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

    void clean_bitmask() {
        for( int i = 0; i < int(bitmask_.size()); ++i )
            delete[] bitmask_[i];
        bitmask_.clear();
    }
    void set_iterated_level(int iterated_level) {
        iterated_level_ = iterated_level;
        clean_bitmask();
        bitmask_ = vector<unsigned*>(iterated_level_ + 1);
        for( int i = 0; i <= iterated_level_; ++i )
            bitmask_[i] = new unsigned[16];
    }
    void set_inverse_check(bool inverse_check) {
        inverse_check_ = inverse_check;
    }

    bool kappa_ac3() {
        vector<int> revised_vars;
        bool something_removed = CSP::iterated_weighted_arc_consistency_t<kappa_varset_beam_t>::weighted_ac3(iterated_level_, revised_vars, true, inverse_check_);
        normalize_kappas(revised_vars);
        calculate_normalization_constant(revised_vars);
        assert(normalized_kappas());
        return something_removed;
    }

    void calculate_normalization_constant(int loc) {
        domain_[loc]->calculate_normalization_constant();
    }
    void calculate_normalization_constant(const vector<int> &revised_vars) {
        for( int i = 0; i < int(revised_vars.size()); ++i )
            calculate_normalization_constant(revised_vars[i]);
    }

    void normalize_kappas(int loc) {
        domain_[loc]->normalize();
    }
    void normalize_kappas(const vector<int> &revised_vars) {
        for( int i = 0; i < int(revised_vars.size()); ++i )
            normalize_kappas(revised_vars[i]);
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

    void print_factor(ostream &os, int loc) const {
        const kappa_varset_beam_t &beam = *domain_[loc];
        os << "loc=" << loc << ",sz=" << beam.size() << ",domain=" << beam;
    }
    void print(ostream &os) const {
        for( int loc = 0; loc < int(domain_.size()); ++loc ) {
            os << "[";
            print_factor(os, loc);
            os << "]" << endl;
        }
    }
};

// static members for kappa_arc_consistency
CSP::constraint_digraph_t kappa_arc_consistency_t::cg_;

// abstract tracking algorithm
class tracking_t {
  protected:
    // game parameters
    int nrows_;
    int ncols_;
    int nmines_;
    bool noisy_;

    // statistics
    mutable int ngames_;
    mutable int nwins_;
    mutable int nguesses_;
    mutable int ndecisions_;
    mutable int ninferences_;
    mutable float elapsed_time_;
 
  public:
    tracking_t(int nrows, int ncols, int nmines, bool noisy)
      : nrows_(nrows), ncols_(ncols), nmines_(nmines), noisy_(noisy) {
    }
    virtual ~tracking_t() { }

    bool is_flag_action(int action) const {
        return action < nrows_ * ncols_ ? true : false;
    }
    int get_cell(int action) const {
        return is_flag_action(action) ? action : action - (nrows_ * ncols_);
    }

    void initialize_stats() const {
        ngames_ = 0;
        nwins_ = 0;
        nguesses_ = 0;
        ndecisions_ = 0;
        ninferences_ = 0;
        elapsed_time_ = 0;
    }
    void print_stats(ostream &os) const {
          os << "stats: #games=" << ngames_
             << ", #wins=" << nwins_
             << ", %win=" << float(nwins_) / float(ngames_)
             << ", #guesses=" << nguesses_
             << ", #decisions=" << ndecisions_
             << ", #inferences=" << ninferences_
             << ", etime=" << elapsed_time_
             << ", etime/game=" << elapsed_time_ / float(ngames_)
             << ", etime/decision=" << elapsed_time_ / float(ndecisions_)
             << ", etime/inference=" << elapsed_time_ / float(ninferences_)
             << endl;
    }

    void increase_elapsed_time(float time) {
        elapsed_time_ += time;
    }
    void increase_wins() {
        ++nwins_;
    }

    virtual string id() const = 0;
    virtual void reset() = 0;
    virtual void update(bool flag_action, int cell, int obs) = 0;
    virtual int get_action() const = 0;
};

// standard belief tracking for minesweeper
class pbt_t : public tracking_t {
    // variables, factors and centers for each beam
    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;
    vector<int> centers_;

    // computation of marginals in factor model
    mutable vector<int> indices_for_updated_factors_;
    mutable vector<dai::Factor> marginals_;

    // inference algorithm and parameters
    Inference::inference_t inference_;

    // variables for game play
    set<int> plays_;
    int nflags_;

    // for action selection
    mutable float best_prob_for_open_;
    mutable float best_prob_for_flag_;
    mutable vector<int> best_for_open_;
    mutable vector<int> best_for_flag_;

  public:
    pbt_t(int nrows, int ncols, int nmines, bool noisy, const multimap<string, string> parameters)
      : tracking_t(nrows, ncols, nmines, noisy) {
        // create binary variables for each cell in the grid
        variables_ = vector<dai::Var>(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc )
            variables_[loc] = dai::Var(loc, 2);

        // create one factor for each location. The variables
        // in the factor are the variables for the location
        // surrounding the factor, including the variable
        // for the "center" location. Also set up the center
        // for each factor.
        centers_ = vector<int>(nrows_ * ncols_);
        factors_ = vector<dai::Factor>(nrows_ * ncols_);
        marginals_ = vector<dai::Factor>(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            int row = loc / ncols_, col = loc % ncols_;
            vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    if( (dr == 0) && (dc == 0) ) centers_[loc] = vars.size();
                    vars.push_back(variables_[nr * ncols_ + nc]);
                }
            }
            dai::VarSet varset(vars.begin(), vars.end());
            factors_[loc] = dai::Factor(varset);
            marginals_[loc] = dai::Factor(variables_[loc]);
            //cout << "Factor[row=" << row << ",col=" << col << "]=" << p << endl;
        }
        cout << "pbt: factor graph:"
             << " #variables=" << variables_.size()
             << ", #factors=" << factors_.size()
             << endl;

        multimap<string, string>::const_iterator it = parameters.find("inference");
        if( it != parameters.end() ) {
            inference_.set_inference_algorithm(it->second, "MAR", true);
            inference_.create_and_initialize_algorithm(factors_);
        }
        it = parameters.find("edbp-max-iter");
        if( it != parameters.end() )
            inference_.edbp_max_iter_ = strtoul(it->second.c_str(), 0, 0);
    }
    virtual ~pbt_t() { }

    virtual string id() const {
        string id_str;
        id_str = string("pbt()");
        return id_str;
    }

    // reset all factors and game-play variables
    virtual void reset() {
        indices_for_updated_factors_.clear();
        indices_for_updated_factors_.reserve(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / (1 << factor.vars().size());
            for( int j = 0; j < (1 << factor.vars().size()); ++j )
                factor.set(j, p);
            indices_for_updated_factors_.push_back(loc);
        }
        plays_.clear();
        nflags_ = 0;
        best_for_open_.clear();
        best_for_flag_.clear();
        ++ngames_;
    }

    // update for obtained obs for cell
    void update(bool flag_action, int cell, int obs) {
        assert(plays_.find(cell) == plays_.end());
        plays_.insert(cell);

#ifdef DEBUG
        int row = cell / ncols_, col = cell % ncols_;
        cout << "pbt: flag=" << flag_action
             << ", cell=" << cell << ":(" << col << "," << row << ")"
             << ", obs=" << obs << endl;
#endif

        if( !flag_action ) {
            int center = centers_[cell];
            //cout << "center=" << center << ", var=" << factors_[cell].vars().var(center) << endl;
            dai::Factor &factor = factors_[cell];
            for( int j = 0; j < int(factor.nrStates()); ++j ) {
                int popcount = __builtin_popcount(j);
                if( !noisy_ ) {
                    if( popcount != obs ) factor.set(j, 0);
                } else {
                    // noisy update
                    if( popcount == obs ) {
                        factor.set(j, factor[j] * .95);
                    } else if( (popcount == obs - 1) || (popcount == obs + 1) ) {
                        factor.set(j, factor[j] * .05);
                    } else {
                        factor.set(j, 0);
                    }
                }
                if( (j & (1 << center)) != 0 ) factor.set(j, 0);
            }
            indices_for_updated_factors_.push_back(cell);

            // if obs is 0 and not noisy, neighboring cells don't have mines
            if( !noisy_ && (obs == 0) ) {
                if( best_prob_for_open_ < 1 ) {
                    best_prob_for_open_ = 1;
                    best_for_open_.clear();
                }
                int row = cell / ncols_, col = cell % ncols_;
                for( int dr = -1; dr < 2; ++dr ) {
                    int nr = row + dr;
                    if( (nr < 0) || (nr >= nrows_) ) continue;
                    for( int dc = -1; dc < 2; ++dc ) {
                        int nc = col + dc;
                        if( (dr == 0) && (dc == 0) ) continue;
                        if( (nc < 0) || (nc >= ncols_) ) continue;
                        int nloc = nr * ncols_ + nc;
                        //cout << "adding (" << nc << "," << nr << ")" << endl;
                        best_for_open_.push_back(nloc);
                    }
                }
            }
        } else {
            ++nflags_;
        }
    }

    // recommend action using information in the marginals
    virtual int get_action() const {
        ++ndecisions_;
        int action = -1;
        if( (best_prob_for_open_ == 1) && !best_for_open_.empty() ) {
            int cell = -1;
            while( (cell == -1) && !best_for_open_.empty() ) {
                int index = lrand48() % best_for_open_.size();
                int candidate = best_for_open_[index];
                best_for_open_[index] = best_for_open_.back();
                best_for_open_.pop_back();
                if( plays_.find(candidate) == plays_.end() )
                    cell = candidate;
            }
            if( cell != -1 ) action = cell + (nrows_ * ncols_);
        }

        if( (action == -1) && (best_prob_for_flag_ == 1) && !best_for_flag_.empty() ) {
            int cell = -1;
            while( (cell == -1) && !best_for_flag_.empty() ) {
                int index = lrand48() % best_for_flag_.size();
                int candidate = best_for_flag_[index];
                best_for_flag_[index] = best_for_flag_.back();
                best_for_flag_.pop_back();
                if( plays_.find(candidate) == plays_.end() )
                    cell = candidate;
            }
            if( cell != -1 ) action = cell;
        }

        if( action == -1 ) {
            calculate_marginals();
            best_for_open_.clear();
            best_for_flag_.clear();
            best_prob_for_open_ = 0;
            best_prob_for_flag_ = 0;
            for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
                if( plays_.find(loc) != plays_.end() ) continue; // cell had been already played
                const dai::Factor &marginal = marginals_[loc];
                assert(marginal.nrStates() == 2);
                if( best_for_open_.empty() || (marginal[0] >= best_prob_for_open_) ) {
                    if( best_for_open_.empty() || (marginal[0] > best_prob_for_open_) ) {
                        best_prob_for_open_ = marginal[0];
                        best_for_open_.clear();
                    }
                    best_for_open_.push_back(loc);
                }
                if( (nflags_ < nmines_) && (best_for_flag_.empty() || (marginal[1] >= best_prob_for_flag_)) ) {
                    if( best_for_flag_.empty() || (marginal[1] > best_prob_for_flag_) ) {
                        best_prob_for_flag_ = marginal[1];
                        best_for_flag_.clear();
                    }
                    best_for_flag_.push_back(loc);
                }
            }

            cout << "best for open: p=" << best_prob_for_open_ << ", sz=" << best_for_open_.size() << ", loc=";
            for( int i = 0; i < int(best_for_open_.size()); ++i )
                cout << best_for_open_[i] << ",";
            cout << endl << "best for flag: p=" << best_prob_for_flag_ << ", sz=" << best_for_flag_.size() << ", loc=";
            for( int i = 0; i < int(best_for_flag_.size()); ++i )
                cout << best_for_flag_[i] << ",";
            cout << endl;

            // prioritize open over flag actions
            if( !best_for_open_.empty() && (best_prob_for_open_ >= best_prob_for_flag_) ) {
                nguesses_ += best_prob_for_open_ < 1;
                int index = lrand48() % best_for_open_.size();
                int cell = best_for_open_[index];
                best_for_open_[index] = best_for_open_.back();
                best_for_open_.pop_back();
                action = cell + (nrows_ * ncols_);
            } else if( !best_for_flag_.empty() && (best_prob_for_open_ < best_prob_for_flag_) ) {
                nguesses_ += best_prob_for_flag_ < 1;
                int index = lrand48() % best_for_flag_.size();
                int cell = best_for_flag_[index];
                best_for_flag_[index] = best_for_flag_.back();
                best_for_flag_.pop_back();
                action = cell;
            }
        }

        assert(action != -1);
        return action;
    }

  protected:
    void calculate_marginals(bool print_marginals = false) const {
        ++ninferences_;
        inference_.calculate_marginals(variables_,
                                       indices_for_updated_factors_,
                                       factors_,
                                       marginals_,
                                       0, // not used because type = MAR
                                       print_marginals);
    }
};

class pbt_ac3_t : public tracking_t {
    int iterated_level_;
    bool inverse_check_;
    kappa_arc_consistency_t kappa_csp_;

#if 0
    // variables, factors and centers for each beam
    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;
    vector<int> centers_;

    // computation of marginals in factor model
    mutable vector<int> indices_for_updated_factors_;
    mutable vector<dai::Factor> marginals_;

    // inference algorithm and parameters
    Inference::inference_t inference_;
#endif

    // variables for game play
    set<int> plays_;
    int nflags_;

    // for action selection
    mutable float best_prob_for_open_;
    mutable float best_prob_for_flag_;
    mutable vector<int> best_for_open_;
    mutable vector<int> best_for_flag_;

  public:
    pbt_ac3_t(int nrows, int ncols, int nmines, bool noisy, const multimap<string, string> parameters)
      : tracking_t(nrows, ncols, nmines, noisy), iterated_level_(0), inverse_check_(true) {
        // set beams
        for( int loc = 0; loc < nrows * ncols; ++loc ) {
            kappa_csp_.set_domain(loc, new kappa_varset_beam_t(loc, cache_t::variable(loc), cache_t::varset(loc)));
        }

        // parse parameters
        multimap<string, string>::const_iterator it = parameters.find("level");
        if( it != parameters.end() )
            iterated_level_ = strtoul(it->second.c_str(), 0, 0);
        it = parameters.find("inverse-check");
        if( it != parameters.end() )
            inverse_check_ = it->second == "true";
        kappa_csp_.set_iterated_level(iterated_level_);
        kappa_csp_.set_inverse_check(inverse_check_);
    }
    virtual ~pbt_ac3_t() {
        kappa_csp_.delete_domains_and_clear();
    }

    virtual string id() const {
        string id_str;
        id_str = string("pbt-ac3(")
          + string("level=") + to_string(iterated_level_)
          + string(",inverse-check=") + (inverse_check_ ? "true" : "false")
          + string(")");
        return id_str;
    }

    // reset all factors and game-play variables
    virtual void reset() {
        reset_csp();
        plays_.clear();
        nflags_ = 0;
        best_for_open_.clear();
        best_for_flag_.clear();
        ++ngames_;
#if 0 // REMOVE
        indices_for_updated_factors_.clear();
        indices_for_updated_factors_.reserve(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / (1 << factor.vars().size());
            for( int j = 0; j < (1 << factor.vars().size()); ++j )
                factor.set(j, p);
            indices_for_updated_factors_.push_back(loc);
        }
#endif
    }

    // update for obtained obs for cell
    void update(bool flag_action, int cell, int obs) {
#if 0
        int current_loc = loc_history_.back();
#ifdef DEBUG
        std::cout << "factor before update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  kappa-csp: ";
        kappa_csp_.print_factor(std::cout, current_loc);
        std::cout << std::endl;
#endif
        assert(current_loc < int(kappa_csp_.nvars()));
        std::vector<std::pair<int, int> > kappa_increases;
        kappa_varset_beam_t &beam = *kappa_csp_.domain(current_loc);
        for( kappa_varset_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            int value = *it;
            int index = it.index();
            int slabels = get_slabels(beam, value);
            float p = base_->probability_obs(obs, current_loc, slabels, last_action);
            int k_obs = kappa_t::kappa(p);
            kappa_increases.push_back(std::make_pair(index, k_obs));
        }
#ifdef DEBUG
        std::cout << "  increases:";
        for( int i = 0; i < int(kappa_increases.size()); ++i ) {
            std::pair<int, int> &p = kappa_increases[i];
            int valuation = beam[p.first].first;
            int kappa = beam[p.first].second;
            std::cout << " " << p.second;
        }
        std::cout << std::endl;
#endif

        assert(!beam.empty());
        for( int i = 0; i < int(kappa_increases.size()); ++i )
            kappa_csp_.domain(current_loc)->increase_kappa(kappa_increases[i]);
        assert(kappa_csp_.worklist().empty());
        kappa_csp_.add_to_worklist(current_loc);
        kappa_csp_.normalize_kappas(current_loc);
        kappa_csp_.kappa_ac3();
        kappa_csp_.normalize_kappas(current_loc);
        kappa_csp_.calculate_normalization_constant(current_loc);

#ifdef DEBUG
        std::cout << "factor after update: loc=" << current_loc << ", obs=" << obs << std::endl;
        std::cout << "  kappa-csp: ";
        kappa_csp_.print_factor(std::cout, current_loc);
        std::cout << std::endl;
#endif
        return kappa_csp_.is_consistent(0);
#endif

        assert(plays_.find(cell) == plays_.end());
        plays_.insert(cell);

#ifdef DEBUG
        int row = cell / ncols_, col = cell % ncols_;
        cout << "pbt-ac3: flag=" << flag_action
             << ", cell=" << cell << ":(" << col << "," << row << ")"
             << ", obs=" << obs << endl;
#endif

        if( !flag_action ) {
            assert(0);
#if 0
            int center = centers_[cell];
            //cout << "center=" << center << ", var=" << factors_[cell].vars().var(center) << endl;
            dai::Factor &factor = factors_[cell];
            for( int j = 0; j < int(factor.nrStates()); ++j ) {
                int popcount = __builtin_popcount(j);
                if( !noisy_ ) {
                    if( popcount != obs ) factor.set(j, 0);
                } else {
                    // noisy update
                    if( popcount == obs ) {
                        factor.set(j, factor[j] * .95);
                    } else if( (popcount == obs - 1) || (popcount == obs + 1) ) {
                        factor.set(j, factor[j] * .05);
                    } else {
                        factor.set(j, 0);
                    }
                }
                if( (j & (1 << center)) != 0 ) factor.set(j, 0);
            }
            indices_for_updated_factors_.push_back(cell);
#endif

            // if obs is 0 and not noisy, neighboring cells don't have mines
            if( !noisy_ && (obs == 0) ) {
                if( best_prob_for_open_ < 1 ) {
                    best_prob_for_open_ = 1;
                    best_for_open_.clear();
                }
                int row = cell / ncols_, col = cell % ncols_;
                for( int dr = -1; dr < 2; ++dr ) {
                    int nr = row + dr;
                    if( (nr < 0) || (nr >= nrows_) ) continue;
                    for( int dc = -1; dc < 2; ++dc ) {
                        int nc = col + dc;
                        if( (dr == 0) && (dc == 0) ) continue;
                        if( (nc < 0) || (nc >= ncols_) ) continue;
                        int nloc = nr * ncols_ + nc;
                        //cout << "adding (" << nc << "," << nr << ")" << endl;
                        best_for_open_.push_back(nloc);
                    }
                }
            }
        } else {
            ++nflags_;
        }
    }

    // recommend action using information in the marginals
    virtual int get_action() const {
        ++ndecisions_;
        int action = -1;
        if( (best_prob_for_open_ == 1) && !best_for_open_.empty() ) {
            int cell = -1;
            while( (cell == -1) && !best_for_open_.empty() ) {
                int index = lrand48() % best_for_open_.size();
                int candidate = best_for_open_[index];
                best_for_open_[index] = best_for_open_.back();
                best_for_open_.pop_back();
                if( plays_.find(candidate) == plays_.end() )
                    cell = candidate;
            }
            if( cell != -1 ) action = cell + (nrows_ * ncols_);
        }

        if( (action == -1) && (best_prob_for_flag_ == 1) && !best_for_flag_.empty() ) {
            int cell = -1;
            while( (cell == -1) && !best_for_flag_.empty() ) {
                int index = lrand48() % best_for_flag_.size();
                int candidate = best_for_flag_[index];
                best_for_flag_[index] = best_for_flag_.back();
                best_for_flag_.pop_back();
                if( plays_.find(candidate) == plays_.end() )
                    cell = candidate;
            }
            if( cell != -1 ) action = cell;
        }

        if( action == -1 ) {
            assert(0);
#if 0
            calculate_marginals();
#endif
            best_for_open_.clear();
            best_for_flag_.clear();
            best_prob_for_open_ = 0;
            best_prob_for_flag_ = 0;
            for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
                if( plays_.find(loc) != plays_.end() ) continue; // cell had been already played
#if 0
                const dai::Factor &marginal = marginals_[loc];
                assert(marginal.nrStates() == 2);
                if( best_for_open_.empty() || (marginal[0] >= best_prob_for_open_) ) {
                    if( best_for_open_.empty() || (marginal[0] > best_prob_for_open_) ) {
                        best_prob_for_open_ = marginal[0];
                        best_for_open_.clear();
                    }
                    best_for_open_.push_back(loc);
                }
#endif
#if 0
                if( (nflags_ < nmines_) && (best_for_flag_.empty() || (marginal[1] >= best_prob_for_flag_)) ) {
                    if( best_for_flag_.empty() || (marginal[1] > best_prob_for_flag_) ) {
                        best_prob_for_flag_ = marginal[1];
                        best_for_flag_.clear();
                    }
                    best_for_flag_.push_back(loc);
                }
#endif
            }

            cout << "best for open: p=" << best_prob_for_open_ << ", sz=" << best_for_open_.size() << ", loc=";
            for( int i = 0; i < int(best_for_open_.size()); ++i )
                cout << best_for_open_[i] << ",";
            cout << endl << "best for flag: p=" << best_prob_for_flag_ << ", sz=" << best_for_flag_.size() << ", loc=";
            for( int i = 0; i < int(best_for_flag_.size()); ++i )
                cout << best_for_flag_[i] << ",";
            cout << endl;

            // prioritize open over flag actions
            if( !best_for_open_.empty() && (best_prob_for_open_ >= best_prob_for_flag_) ) {
                nguesses_ += best_prob_for_open_ < 1;
                int index = lrand48() % best_for_open_.size();
                int cell = best_for_open_[index];
                best_for_open_[index] = best_for_open_.back();
                best_for_open_.pop_back();
                action = cell + (nrows_ * ncols_);
            } else if( !best_for_flag_.empty() && (best_prob_for_open_ < best_prob_for_flag_) ) {
                nguesses_ += best_prob_for_flag_ < 1;
                int index = lrand48() % best_for_flag_.size();
                int cell = best_for_flag_[index];
                best_for_flag_[index] = best_for_flag_.back();
                best_for_flag_.pop_back();
                action = cell;
            }
        }

        assert(action != -1);
        return action;
    }

  protected:
    void reset_csp() {
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            kappa_csp_.domain(loc)->set_initial_configuration();
            kappa_csp_.calculate_normalization_constant(loc);
        }
        assert(kappa_csp_.normalized_kappas());
    }
};

struct cell_t {
    bool mine_;
    int nmines_;
    cell_t() : mine_(false), nmines_(0) { }
};

class minefield_t {
  protected:
    int nrows_;
    int ncols_;
    int ncells_;
    int nmines_;
    int num_remaining_mines_;

    vector<cell_t> cells_;

  public:
    minefield_t(int nrows, int ncols, int nmines)
      : nrows_(nrows), ncols_(ncols), ncells_(nrows_ * ncols_), nmines_(nmines) {
    }
    ~minefield_t() { }

    void sample(int initial_cell) {
        // do not place a mine at the initial cell or surrounding cells
        // calculate places where a mine can be placed
        vector<int> available_cells(ncells_, 0);
        set<int> forbidden;
        do {
            forbidden.clear();
            available_cells = vector<int>(ncells_, 0);
            for( int i = 0; i < ncells_; ++i )
                available_cells[i] = i;

            int r = initial_cell / ncols_, c = initial_cell % ncols_;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = r + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = c + dc;
                    int cell = nr * ncols_ + nc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    forbidden.insert(-(1 + cell));
                }
            }
            for( set<int>::iterator it = forbidden.begin(); it != forbidden.end(); ++it ) {
                int pos = -(*it) - 1;
                available_cells[pos] = available_cells.back();
                available_cells.pop_back();
            }
        } while( (int)available_cells.size() < nmines_ );

        // available_cells contains the cells where mines can be placed.
        // Place mines in random cells. For each placed mines, increase
        // counter of surronding mines in surrounding cells.
        cells_ = vector<cell_t>(ncells_);
        for( int num_placed_mines = 0; num_placed_mines < nmines_; ++num_placed_mines ) {
            int pos = lrand48() % available_cells.size();
            int cell = available_cells[pos];
            available_cells[pos] = available_cells.back();
            available_cells.pop_back();
            assert(!cells_[cell].mine_);
            cells_[cell].mine_ = true;
            int r = cell / ncols_, c = cell % ncols_;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = r + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = c + dc;
                    int ncell = nr * ncols_ + nc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    if( cell == ncell ) continue;
                    ++cells_[ncell].nmines_;
                }
            }
        }
        num_remaining_mines_ = nmines_;
    }

    int apply_action(const tracking_t &tracker, int action, bool verbose) {
        int cell = tracker.get_cell(action);
        if( tracker.is_flag_action(action) ) {
            return flag_cell(cell, verbose);
        } else {
            return open_cell(cell, verbose);
        }
    }

    bool is_mine(int cell) const {
        return cells_[cell].mine_;
    }

    void print(ostream &os, bool formatted = false) const {
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                if( cells_[cell].mine_ )
                    os << " *";
                else
                    os << " " << cells_[cell].nmines_;
            }
            if( formatted ) os << endl;
        }
    }

  protected:
    int num_remaining_mines() const {
        return num_remaining_mines_;
    }

    int flag_cell(int cell, bool verbose) {
        int r = cell / ncols_, c = cell % ncols_;
        if( verbose ) cout << "flag_cell(" << c << "," << r << "): mine=" << (cells_[cell].mine_ ? 1 : 0) << endl;
        num_remaining_mines_ -= cells_[cell].mine_ ? 1 : 0;
        return -1;
    }

    int open_cell(int cell, bool verbose) {
        int r = cell / ncols_, c = cell % ncols_;
        if( verbose ) cout << "open_cell(" << c << "," << r << "): mine=" << (cells_[cell].mine_ ? 1 : 0) << ", #mines=" << cells_[cell].nmines_ << endl;
        return cells_[cell].mine_ ? 9 : cells_[cell].nmines_;
    }
};

#if 0
// divergence measures between distributions
pair<float, bool> kl_divergence(const vector<float> &P, const vector<float> &Q) {
    assert(P.size() == Q.size());
    float kl = 0;
    for( size_t i = 0; i < P.size(); ++i ) {
        float p = P[i];
        float q = Q[i];
        if( (q == 0) && (p != 0) ) return make_pair(kl, false);
        if( p == 0 ) continue;
        kl += p * (log(p) - log(q));
    }
    return make_pair(kl, true);
}

pair<float, bool> js_divergence(const vector<float> &P, const vector<float> &Q) {
    assert(P.size() == Q.size());
    vector<float> M(P.size(), 0);
    for( size_t i = 0; i < M.size(); ++i )
        M[i] = (P[i] + Q[i]) / 2;
    pair<float, bool> kl1 = kl_divergence(P, M);
    pair<float, bool> kl2 = kl_divergence(Q, M);
    assert(kl1.second && kl2.second);
    return make_pair((kl1.first + kl2.first) / 2, true);
}
#endif

void usage(ostream &os) {
    os << endl
       << "Usage: mines [{-t | --ntrials} <ntrials>]" << endl
       << "             [{-r | --nrows} <nrows>]" << endl
       << "             [{-c | --ncols} <ncols>]" << endl
       << "             [{-m | --nmines} <nmines>]" << endl
       << "             [{-s | --seed} <seed>]" << endl
       << "             [{-v | --verbose}]" << endl
       << "             [{-? | --help}]" << endl
       << endl
       << "where <ntrials> is a non-negative integer telling the number of games to" << endl
       << "play (default is 1), <nrows> and <ncols> are positive integers telling" << endl
       << "the dimensions of the minefield (default is 16x16), <nmines> is a positive" << endl
       << "integer telling the number of hidden mines in the minefield (default is 40)," << endl
       << "and <seed> is an integer for setting the seed of the random number generator" << endl
       << "(default is 0)." << endl
       << endl
       << "For example," << endl
       << endl
       << "  ./mines -r 16 -c 16 -m 40 -t 100" << endl
       << endl
       << "performs an experiment consisting of 100 trials on a 16x16 minefield" << endl
       << "with 40 mines." << endl
       << endl;
}

void finalize() {
}

int main(int argc, const char **argv) {
    int ntrials = 1;
    int nrows = 16;
    int ncols = 16;
    int nmines = 40;
    int seed = 0;
    bool verbose = false;
    float epsilon_for_kappa = 0.1;
    string tmp_path = "";
    vector<string> tracker_strings;

    // parse arguments
    for( --argc, ++argv; (argc > 0) && (**argv == '-'); --argc, ++argv ) {
        if( !strcmp(argv[0], "--tmp-path") ) {
            tmp_path = argv[1];
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-t") || !strcmp(argv[0], "--ntrials") ) {
            ntrials = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-r") || !strcmp(argv[0], "--nrows") ) {
            nrows = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-c") || !strcmp(argv[0], "--ncols") ) {
            ncols = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-m") || !strcmp(argv[0], "--nmines") ) {
            nmines = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strncmp(argv[0], "--tracker=", 10) ) {
            string tracker(&argv[0][10]);
            Utils::tokenize(tracker, tracker_strings);
        } else if( !strcmp(argv[0], "-v") || !strcmp(argv[0], "--verbose") ) {
            verbose = true;
        } else if( !strcmp(argv[0], "-?") || !strcmp(argv[0], "--help") ) {
            usage(cout);
            exit(-1);
        } else {
            cout << "error: unexpected argument: " << argv[0] << endl;
        }
    }

    // set seed
    Utils::set_seed(seed);
    cout << "# seed=" << seed << endl;

    // set static members
    if( !tmp_path.empty() && (tmp_path.back() != '/') ) tmp_path += '/';
    //Inference::edbp_t::initialize();
    //Inference::inference_t::initialize_edbp(tmp_path);
    kappa_t::initialize(epsilon_for_kappa, 10);

    // tracking algorithms
    vector<tracking_t*> trackers;
    for( size_t i = 0; i < tracker_strings.size(); ++i ) {
        tracking_t *tracker = 0;
        const string &name = tracker_strings[i];
        string short_name;
        string parameter_str;
        multimap<string, string> parameters;
        Utils::split_request(name, short_name, parameter_str);
        Utils::tokenize(parameter_str, parameters);
        if( short_name == "pbt" ) {
            tracker = new pbt_t(nrows, ncols, nmines, false, parameters);
        } else if( short_name == "pbt-ac3" ) {
            tracker = new pbt_ac3_t(nrows, ncols, nmines, false, parameters);
        } else {
            cerr << "warning: unrecognized tracking algorithm '" << name << "'" << endl;
        }
        if( tracker != 0 ) trackers.push_back(tracker);
    }

    // check that there is something to do
    if( trackers.empty() ) {
        cout << "warning: no tracker specified. Terminating..." << endl;
        finalize();
        return 0;
    }

    // print identity of trackers
    for( size_t i = 0; i < trackers.size(); ++i )
        cout << "# tracker[" << i << "].id=\"" << trackers[i]->id() << "\"" << endl;
    
    // initialize stats for trackers
    for( size_t i = 0; i < trackers.size(); ++i )
        trackers[i]->initialize_stats();

    // run for the specified number of trials
    for( int trial = 0; trial < ntrials; ) {
        for( int i = 0; i < int(trackers.size()); ++i ) {
            minefield_t minefield(nrows, ncols, nmines);
            tracking_t &tracker = *trackers[i];
            tracker.reset();
            float start_time = Utils::read_time_in_seconds();

            bool win = true;
            vector<pair<int,int> > execution(nrows * ncols);
            for( int play = 0; play < nrows * ncols; ++play ) {
                int action = tracker.get_action();
                bool is_flag_action = tracker.is_flag_action(action);
                int cell = tracker.get_cell(action);
                cout << "Play: n=" << play
                     << ", type=" << (is_flag_action ? "FLAG" : "OPEN")
                     << ", cell=" << cell << ":(" << cell % ncols << "," << cell / ncols << ")"
                     << flush;

                // if this is first play, then it must be an open action. The minefield
                // gets sampled using the action's cell.
                if( play == 0 ) {
                    assert(!tracker.is_flag_action(action));
                    int cell = tracker.get_cell(action);
                    minefield.sample(cell);
                    assert(!minefield.is_mine(cell));
                    cout << ", obs=0" << endl << endl;
                    minefield.print(cout, true);
                    cout << endl;
                }

                // obtain observation for this action. If first play, observation
                // must be zero.
                int obs = minefield.apply_action(tracker, action, verbose);
                assert((obs == 0) || (play > 0));
                if( play > 0 ) cout << ", obs=" << obs << endl;
                if( obs == 9 ) {
                    cout << "**** BOOM!!! ****" << endl;
                    win = false;
                    break;
                }

                // update execution and agent's belief 
                execution[play] = make_pair(action, obs);

                // update agent's belief
                tracker.update(tracker.is_flag_action(action), tracker.get_cell(action), obs);
            }
            tracker.increase_elapsed_time(Utils::read_time_in_seconds() - start_time);

            if( win ) tracker.increase_wins();
            tracker.print_stats(cout);
            ++trial;
        }
    }
    return 0;
}

