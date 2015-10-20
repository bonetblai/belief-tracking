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

#ifndef SIR_H
#define SIR_H

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

#include "particle_filter.h"

struct history_container_t : public std::map<std::vector<int>, int> {
};

// Generic Sequential Importance Resampling (SIR) Particle Filter
template <typename PTYPE, typename BASE> struct SIR_t : public PF_t<PTYPE, BASE> {
    using tracking_t<BASE>::base_;
    using PF_t<PTYPE, BASE>::nparticles_;
    using PF_t<PTYPE, BASE>::particles_;
    using PF_t<PTYPE, BASE>::multiplicity_;
    using PF_t<PTYPE, BASE>::marginals_on_vars_;

    bool do_stochastic_universal_sampling_;
    std::vector<std::pair<int, int> > execution_;

    SIR_t(const std::string &name, const BASE &base, int nparticles, bool do_stochastic_universal_sampling = false)
      : PF_t<PTYPE, BASE>(name, base, nparticles), do_stochastic_universal_sampling_(do_stochastic_universal_sampling) {
    }
    virtual ~SIR_t() { }

    virtual void sample_from_pi(PTYPE &np, const PTYPE &p, int last_action, int obs, const history_container_t &history_container, int pindex) const = 0;
    virtual float importance_weight(const PTYPE &np, const PTYPE &p, int last_action, int obs) const = 0;

    virtual void initialize() {
        PTYPE sampler;
        float weight = 1.0 / float(nparticles_);
#ifndef USE_MPI
        PTYPE *p = sampler.initial_sampling();
#else
        assert(mpi_base_t::mpi_ != 0);
        assert(mpi_base_t::mpi_->nworkers_ >= 1);
        PTYPE *p = sampler.initial_sampling(mpi_base_t::mpi_, 1);
#endif
        particles_.push_back(std::make_pair(weight, p));
        multiplicity_.push_back(nparticles_);

#ifdef USE_MPI 
        // process was lauched to calculate marginals, collect marginals
        particles_[0].second->mpi_update_marginals(mpi_base_t::mpi_, 1);
#endif
    }

    virtual void update(int last_action, int obs) {
        std::vector<int> indices;
        if( do_stochastic_universal_sampling_ )
            PF_t<PTYPE, BASE>::stochastic_universal_sampling(nparticles_, indices);
        else
            PF_t<PTYPE, BASE>::stochastic_sampling(nparticles_, indices);
        assert(int(indices.size()) == nparticles_);

        history_container_t history_container;
        std::vector<std::pair<float, PTYPE*> > new_particles;
        for( int i = 0; i < int(indices.size()); ++i ) {
            int index = indices[i];
            const PTYPE &p = *particles_[index].second;
            float weight = particles_[index].first;

#ifndef USE_MPI
            PTYPE *np = sample_from_pi(p, last_action, obs, history_container);
#else
            PTYPE *np = sample_from_pi(p, last_action, obs, history_container, 1 + i);
#endif

            if( history_container.find(np->history()) == history_container.end() ) {
                history_container.insert(std::make_pair(np->history(), 1));
            } else {
                ++history_container[np->history()];
                delete np;
                continue;
            }

            float new_weight = weight * importance_weight(*np, p, last_action, obs);
            new_particles.push_back(std::make_pair(new_weight, np));
        }

        // set multiplicity and normalize
        int nparticles = 0;
        float total_mass = 0.0;
        multiplicity_ = std::vector<int>(new_particles.size(), 0);
        for( int i = 0; i < int(new_particles.size()); ++i ) {
            assert(history_container.find(new_particles[i].second->history()) != history_container.end());
            multiplicity_[i] = history_container.find(new_particles[i].second->history())->second;
            total_mass += new_particles[i].first * multiplicity_[i];
            nparticles += multiplicity_[i];
        }
        assert(nparticles_ == nparticles);

        for( int i = 0; i < int(new_particles.size()); ++i ) {
            if( total_mass == 0 )
                new_particles[i].first = 1.0 / float(new_particles.size());
            else
                new_particles[i].first /= total_mass;
        }

        // replace old particles by new particles
        for( int i = 0; i < int(particles_.size()); ++i )
            delete particles_[i].second;
        particles_ = std::move(new_particles);

        execution_.push_back(std::make_pair(last_action, obs));

#ifdef USE_MPI 
        // processes were lauched to calculate marginals, now collect marginals
        assert(mpi_base_t::mpi_ != 0);
        assert(mpi_base_t::mpi_->nworkers_ >= 1);
        for( int i = 0; i < int(new_particles.size()); ++i ) {
            PTYPE *p = particles_[i].second;
            p->mpi_update_marginals(mpi_base_t::mpi_, 1 + i);
        }   
#endif
    }

    virtual void calculate_marginals() {
        // initialize marginals
        marginals_on_vars_ = std::vector<dai::Factor>(base_.nvars_);
        for( int var = 0; var < base_.nvars_; ++var ) {
            marginals_on_vars_[var] = dai::Factor(dai::VarSet(dai::Var(var, base_.variable_size(var))), 0.0);
        }

        // aggregate info from particles into marginals
        for( int i = 0; i < nparticles_; ++i ) {
            float weight = particles_[i].first;
            const PTYPE &p = *particles_[i].second;
            for( int var = 0; var < base_.nvars_; ++var ) {
                marginals_on_vars_[var].set(p.value_for(var), marginals_on_vars_[var][p.value_for(var)] + weight);
            }
        }
    }

    virtual void get_marginal(int var, dai::Factor &marginal) const {
        marginal = marginals_on_vars_[var];
    }

    virtual float* get_marginal(int var, float *ptr) const {
        const dai::Factor &factor = marginals_on_vars_[var];
        for( int value = 0; value < int(factor.nrStates()); ++value )
            *ptr++ = factor[value];
        return ptr;
    }

    PTYPE* sample_from_pi(const PTYPE &p, int last_action, int obs, const history_container_t &history_container, int pindex = -1) const {
        PTYPE *np = new PTYPE(p);
        sample_from_pi(*np, p, last_action, obs, history_container, pindex);
        return np;
    }

    virtual std::string id() const {
        std::string id_str;
        id_str = std::string("PF(type=SIR,nparticles=") + std::to_string(nparticles_)
          //+ ",sus=" + std::to_string(do_stochastic_universal_sampling_)
          + ")";
        return id_str;
    }
};

#endif

