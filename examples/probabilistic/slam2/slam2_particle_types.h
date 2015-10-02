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

    // inference type and tmp filenames
    int inference_type_;
    std::string edbp_tmp_fn_;
    std::string edbp_evid_fn_;

    // variables and factors
    std::vector<dai::Factor> factors_;

    // computation of marginal in factor model
    mutable std::vector<int> indices_for_updated_factors_;
    mutable std::vector<dai::Factor> marginals_;
    dai::InfAlg *inference_algorithm_;

    // conversion between libdai and edbp factors
    static std::vector<std::vector<int> > edbp_factor_indices_;

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

        // create inference algorithm
        create_and_initialize_inference_algorithm();
    }
    virtual ~rbpf_slam2_particle_t() {
        destroy_inference_algorithm();
    }

    rbpf_slam2_particle_t(const rbpf_slam2_particle_t &p) {
        *this = p;
    }

    rbpf_slam2_particle_t(rbpf_slam2_particle_t &&p)
      : loc_history_(p.loc_history_),
        inference_type_(p.inference_type_),
        edbp_tmp_fn_(p.edbp_tmp_fn_),
        edbp_evid_fn_(p.edbp_evid_fn_),
        factors_(p.factors_),
        indices_for_updated_factors_(p.indices_for_updated_factors_),
        marginals_(p.marginals_) {
        inference_algorithm_ = p.inference_algorithm_;
        p.inference_algorithm_ = 0;
    }

    const rbpf_slam2_particle_t& operator=(const rbpf_slam2_particle_t &p) {
        loc_history_ = p.loc_history_;
        inference_type_ = p.inference_type_;
        edbp_tmp_fn_ = p.edbp_tmp_fn_;
        edbp_evid_fn_ = p.edbp_evid_fn_;
        factors_ = p.factors_;
        indices_for_updated_factors_ = p.indices_for_updated_factors_;
        marginals_ = p.marginals_;
        if( p.inference_algorithm_ != 0 )
            inference_algorithm_ = p.inference_algorithm_->clone();
        else
            inference_algorithm_ = 0;
        return *this;
    }

    bool operator==(const rbpf_slam2_particle_t &p) const {
        return (loc_history_ == p.loc_history_) && (factors_ == p.factors_) &&
               (inference_type_ == p.inference_type_) &&
               (edbp_tmp_fn_ == p.edbp_tmp_fn_) &&
               (edbp_evid_fn_ == p.edbp_evid_fn_) &&
               (factors_ == p.factors_) &&
               (indices_for_updated_factors_ == p.indices_for_updated_factors_) &&
               (marginals_ == p.marginals_) &&
               (((inference_algorithm_ == 0) && (p.inference_algorithm_ == 0)) ||
                ((inference_algorithm_ != 0) && (p.inference_algorithm_ != 0)));
    }

    static void compute_edbp_factor_indices() {
        edbp_factor_indices_ = std::vector<std::vector<int> >(10);

        // define indices to compute
        std::vector<int> number_vars;
        number_vars.push_back(2);
        number_vars.push_back(3);
        number_vars.push_back(4);
        number_vars.push_back(5);
        number_vars.push_back(6);
        number_vars.push_back(9);

        for( int i = 0; i < int(number_vars.size()); ++i ) {
            int nvars = number_vars[i];
            edbp_factor_indices_[nvars] = std::vector<int>(1 << nvars);

            dai::VarSet vars, r_vars;
            for( int j = 0; j < nvars; ++j ) {
                vars |= dai::Var(j, 2);
                r_vars |= dai::Var(nvars - j - 1, 2);
            }
            //std::cout << "vars=" << vars << ", r_vars=" << r_vars << std::endl;

            for( int j = 0; j < (1 << nvars); ++j ) {
                std::map<dai::Var, size_t> state = dai::calcState(vars, j);
                std::map<dai::Var, size_t> r_state;
                for( std::map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it ) {
                    int label = nvars - 1 - it->first.label();
                    r_state[dai::Var(label, 2)] = it->second;
                }
                int r_index = dai::calcLinearState(r_vars, r_state);
                edbp_factor_indices_[nvars][r_index] = j;
#if 0
                std::cout << "State of vars(index=" << j << "): " //<< state
                          << "; state of r_vars(index=" << r_index << "): " //<< dai::calcState(r_vars, r_index)
                          << std::endl;
#endif
            }
        }
    }
    static int edbp_factor_index(int nvars, int index) {
        assert(nvars < int(edbp_factor_indices_.size()));
        assert(index < int(edbp_factor_indices_[nvars].size()));
        return edbp_factor_indices_[nvars][index];
    }

    void create_and_initialize_inference_algorithm() {
        inference_type_ = 0;
        dai::FactorGraph factor_graph(factors_);

        if( inference_type_ == 0 ) {
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
        } else if( inference_type_ == 1 ) {
            edbp_tmp_fn_ = std::string("dummy.uai");
            edbp_evid_fn_ = std::string("dummy.evid");
        }
    }

    void destroy_inference_algorithm() {
        delete inference_algorithm_;
        inference_algorithm_ = 0;
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
            os << ":  j=" << std::setw(3) << j
               << ",  nbits=" << base_->num_bits(j)
               << ",  value=" << factor[j]
               << std::endl;
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
        indices_for_updated_factors_.clear();
        indices_for_updated_factors_.reserve(base_->nloc_);
        loc_history_.push_back(base_->initial_loc_);
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / (1 << factor.vars().size());
            for( int i = 0; i < (1 << factor.vars().size()); ++i )
                factor.set(i, p);
            indices_for_updated_factors_.push_back(loc);
        }
        calculate_marginals(); // CHECK: can this be postponed?
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

    void update_factors(int last_action, int obs) {
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
        indices_for_updated_factors_.push_back(current_loc);
#ifdef DEBUG
        std::cout << "factor after  update: loc=" << current_loc << ", obs=" << obs << std::endl;
        print_factor(std::cout, current_loc, factors_[current_loc], "factors_");
#endif
    }

    void calculate_marginals(bool print_marginals = false) const {
        if( !indices_for_updated_factors_.empty() ) {
            if( inference_type_ == 0 ) {
                apply_inference_libdai();
                extract_marginals_from_inference_libdai(print_marginals);
            } else {
                //apply_inference_libdai(); // CHECK
                //extract_marginals_from_inference_libdai(print_marginals); // CHECK
                apply_inference_edbp();
                extract_marginals_from_inference_edbp(print_marginals);
            }
            indices_for_updated_factors_.clear();
        }
    }

    void update_marginals(float weight, std::vector<dai::Factor> &marginals_on_vars) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            dai::Factor marginal = marginals_[loc].marginal(dai::Var(loc, 2));
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

    void apply_inference_libdai() const {
        assert(inference_algorithm_ != 0);
        assert(!indices_for_updated_factors_.empty());
        dai::VarSet variables;
        for( int i = 0; i < int(indices_for_updated_factors_.size()); ++i ) {
            int loc = indices_for_updated_factors_[i];
            inference_algorithm_->fg().setFactor(loc, factors_[loc]);
            variables |= factors_[loc].vars();
            //std::cout << "libdai: updated factor for loc=" << loc << std::endl;
        }
        inference_algorithm_->init(variables);
        inference_algorithm_->run();
    }

    void extract_marginals_from_inference_libdai(bool print_marginals = false) const {
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            marginals_[loc] = inference_algorithm_->belief(factors_[loc].vars());
            if( print_marginals )
                print_factor(std::cout, loc, marginals_[loc], "libdai::marginals_");
        }
    }

    void apply_inference_edbp() const {
        // create model in file dummy.uai
        std::ofstream ofs(edbp_tmp_fn_);
        ofs << "MARKOV"
            << std::endl
            << base_->nloc_
            << std::endl;
        for( int loc = 0; loc < base_->nloc_; ++loc )
            ofs << 2 << (loc < base_->nloc_ - 1 ? " " : "");
        ofs << std::endl << base_->nloc_ << std::endl;
        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            const dai::Factor &factor = factors_[loc];
            ofs << factor.vars().size();
            for( dai::VarSet::const_reverse_iterator it = factor.vars().rbegin(); it != factor.vars().rend(); ++it ) {
                ofs << " " << it->label();
            }
            ofs << std::endl;
        }
        ofs << std::endl;
        for( int loc = 0; loc < int(factors_.size()); ++loc ) {
            print_factor_edbp(ofs, factors_[loc]);
            ofs << std::endl;
        }
        ofs.close();

        // call edbp solver
        std::string edbp_cmd = std::string("~/software/edbp/solver") + " " + edbp_tmp_fn_ + " " + edbp_evid_fn_ + " 0 BEL 2>/dev/null";
        system(edbp_cmd.c_str());
    }

    void extract_marginals_from_inference_edbp(bool print_marginals = false) const {
#if 0
        std::cout << "----------FILE-----------" << std::endl;
        std::string cmd = std::string("cat ") + edbp_tmp_fn_ + ".BEL";
        system(cmd.c_str());
#endif
        //std::vector<dai::Factor> edbp_marginals = marginals_;

        std::ifstream ifs(edbp_tmp_fn_ + ".BEL");

        std::string buff;
        ifs >> buff;
        assert(buff == "BEL");

        //std::cout << "----------BEGIN----------" << std::endl;
        bool more_results = true;
        while( more_results ) {
            int nlines = 0;
            ifs >> nlines;
            assert(nlines == 1);
            //std::cout << "**** nlines=" << nlines << std::endl;

            int nfactors = 0;
            ifs >> nfactors;
            assert(nfactors == base_->nloc_);
            //std::cout << "**** nfactors=" << nfactors << std::endl;
            for( int loc = 0; loc < base_->nloc_; ++loc ) {
                int nvars = factors_[loc].vars().size();
                int nstates = 0;
                ifs >> nstates;
                //std::cout << "**** loc=" << loc << ", nstates=" << nstates << std::endl;
                assert(nstates == (1 << nvars));
                for( int j = 0; j < nstates; ++j ) {
                    float p = 0;
                    ifs >> p;
                    marginals_[loc].set(edbp_factor_index(nvars, j), p);
                    //edbp_marginals[loc].set(edbp_factor_index(nvars, j), p);
                }
            }

            // check if there are more results
            std::string marker;
            ifs >> marker;
            more_results = marker == "-BEGIN-";
            //std::cout << "**** marker=" << marker << ", more-results=" << more_results << std::endl;
        }
        //std::cout << "-----------END-----------" << std::endl;
        ifs.close();

        for( int loc = 0; loc < base_->nloc_; ++loc ) {
            if( print_marginals ) {
                //print_factor(std::cout, loc, factors_[loc], "factors_"); // CHECK
                //print_factor(std::cout, loc, marginals_[loc], "libdai::marginals_"); // CHECK
                print_factor(std::cout, loc, marginals_[loc], "edbp::marginals_");
#if 0
                dai::Factor diff = edbp_marginals[loc] - marginals_[loc];
                std::cout << "diff=" << diff.maxAbs() << std::endl;
                assert(diff.maxAbs() < 1e-5); // CHECK
#endif
            }
        }
    }
};

// Particle for the motion model RBPF filter (slam2)
struct motion_model_rbpf_slam2_particle_t : public rbpf_slam2_particle_t {
    virtual void sample_from_pi(rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        assert(np == p);
        int next_loc = base_->sample_loc(p.loc_history_.back(), last_action);
        np.loc_history_.push_back(next_loc);
        np.update_factors(last_action, obs);
        np.calculate_marginals(false);
    }

    virtual float importance_weight(const rbpf_slam2_particle_t &np, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        assert(p.indices_for_updated_factors_.empty());
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
        assert(p.indices_for_updated_factors_.empty());

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
        std::vector<float> cdf;
        calculate_cdf(p, last_action, obs, cdf);
        int next_loc = Utils::sample_from_distribution(base_->nloc_, &cdf[0]);
        np.loc_history_.push_back(next_loc);
        np.update_factors(last_action, obs);
        np.calculate_marginals(false);
    }

    virtual float importance_weight(const rbpf_slam2_particle_t &/*np*/, const rbpf_slam2_particle_t &p, int last_action, int obs) const {
        assert(p.indices_for_updated_factors_.empty());
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

