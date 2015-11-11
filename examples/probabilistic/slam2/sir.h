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

// Histories are used to detect duplicate particles and correctly
// compute particle multiplicities. History is the value history
// for the tracked variables

typedef std::vector<int> history_t;

struct history_container_t {
    virtual int size() const = 0;
    virtual bool contains(const history_t &history) const = 0;
    virtual void insert(const history_t &history, int index) = 0;
    virtual int multiplicity(const history_t &history) const = 0;
    virtual int index(const history_t &history) const = 0;
    virtual void increase_multiplicity(const history_t &history) = 0;
    virtual void get_multiplicity(std::vector<int> &multiplicity) const = 0;
};

struct standard_history_container_t : public history_container_t {
    std::map<history_t, int> container_;
    std::map<history_t, int> index_;

    virtual int size() const {
        assert(container_.size() == index_.size());
        return container_.size();
    }
    virtual bool contains(const history_t &history) const {
        return container_.find(history) != container_.end();
    }
    virtual void insert(const history_t &history, int index) {
        container_.insert(std::make_pair(history, 1));
        index_.insert(std::make_pair(history, index));
    }
    virtual int multiplicity(const history_t &history) const {
        return container_.find(history)->second;
    }
    virtual int index(const history_t &history) const {
        return index_.find(history)->second;
    }
    virtual void increase_multiplicity(const history_t &history) {
        ++container_[history];
    }
    virtual void get_multiplicity(std::vector<int> &multiplicity) const {
        multiplicity = std::vector<int>(index_.size(), 0);
        for( std::map<history_t, int>::const_iterator it = index_.begin(); it != index_.end(); ++it )
            multiplicity[it->second] = container_.find(it->first)->second;
    }
};

// Generic Sequential Importance Resampling (SIR) Particle Filter
template <typename PTYPE, typename BASE> struct SIR_t : public PF_t<PTYPE, BASE> {
    using tracking_t<BASE>::base_;
    using PF_t<PTYPE, BASE>::nparticles_;
    using PF_t<PTYPE, BASE>::particles_;
    using PF_t<PTYPE, BASE>::multiplicity_;
    using PF_t<PTYPE, BASE>::marginals_on_vars_;
    typedef typename PF_t<PTYPE, BASE>::particle_t particle_t;

    bool do_resampling_;
    bool do_stochastic_universal_sampling_;
    std::vector<std::pair<int, int> > execution_;

#ifdef USE_MPI
    // MPI load balancing
    static int mpi_machine_for_master_;
    static std::vector<std::vector<int> > mpi_fixed_budget_;
#endif

    SIR_t(const std::string &name, const BASE &base, int nparticles, bool do_resampling, bool do_stochastic_universal_sampling)
      : PF_t<PTYPE, BASE>(name, base, nparticles),
        do_resampling_(do_resampling),
        do_stochastic_universal_sampling_(do_stochastic_universal_sampling) {
    }
    virtual ~SIR_t() { }

    virtual void sample_from_pi(PTYPE &np,
                                const PTYPE &p,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                int wid) const = 0;
    virtual float importance_weight(const PTYPE &np,
                                    const PTYPE &p,
                                    int last_action,
                                    int obs) const = 0;

    virtual void initialize() {
        PTYPE sampler;

#ifdef USE_MPI
        // initialize MPI workers
        assert(mpi_base_t::mpi_ != 0);
        assert(mpi_base_t::mpi_->nworkers_ >= nparticles_);
        for( int wid = 0; wid < nparticles_; ++wid )
            sampler.initialize_mpi_worker(mpi_base_t::mpi_, 1 + wid);
#endif

        int wid = 1;
        float weight = 1.0 / float(nparticles_);
        PTYPE *p = sampler.initial_sampling(mpi_base_t::mpi_, wid);
        particles_.push_back(particle_t(weight, p, wid));
        multiplicity_.push_back(nparticles_);

#ifdef USE_MPI 
        // process was lauched to calculate marginals, collect marginals
        particles_[0].p_->mpi_update_marginals(mpi_base_t::mpi_, particles_[0].wid_);
#endif
    }

    // From Grisetti, Stachniss and Burgard. Improved Techniques
    // for Grid Mapping with Rao-Blackwellized Particle Filters
    virtual void update(int last_action, int obs) {
        // 1. Sampling: The next generation of particles is obtained
        // from the current generation by sampling from the proposal
        // distribution \pi.
        //
        // 2. Importance weighting: An individual importance weight
        // is assigned to each (new) particle according to the importance
        // sampling principle:
        //
        // w = p( x(1:t) | z(1:t), u(1:t-1) ) / pi( x(1:t) | z(1:t), u(1:t-1) )
        //
        // The weights account for the fact that the proposal distribution
        // \pi is in general not equal to the target distribution of 
        // successors states.

        standard_history_container_t history_container;
        std::vector<particle_t> new_particles;
        for( int i = 0; i < int(particles_.size()); ++i ) {
            const PTYPE &p = *particles_[i].p_;
            float weight = particles_[i].weight_;
            for( int j = 0; j < multiplicity_[i]; ++j ) {
                int wid = -1;

#ifdef      USE_MPI
                // do load balancing: find wid for machine with lowest load
                int best_mid = -1;
                for( int mid = 0; mid < int(mpi_budget.size()); ++mid ) {
                    if( !mpi_budget[mid].empty() && ((best_mid == -1) || (mpi_load[best_mid] > mpi_load[mid])) )
                        best_mid = mid;
                }
                assert(best_mid >= 0);
                wid = mpi_budget[best_mid].back();
                mpi_budget[best_mid].pop_back();
                ++mpi_load[best_mid];
                //std::cout << "best: mid=" << best_mid << ", wid=" << wid << ", load=" << mpi_load[best_mid] - 1 << std::endl;
#endif

                // sample new particle using the proposal distribution \pi
                PTYPE *np = sample_from_pi(p, last_action, obs, history_container, wid);

                // update histories and multiplicities
                if( !history_container.contains(np->history()) ) {
                    history_container.insert(np->history(), new_particles.size());
                } else {
                    history_container.increase_multiplicity(np->history());
                    delete np;
                    continue;
                }

                // compute new weight using recursive formula and add new particle
                float new_weight = weight * importance_weight(*np, p, last_action, obs);
                new_particles.push_back(particle_t(new_weight, np, wid));
            }
        }
        assert(int(new_particles.size()) == history_container.size());
        std::vector<int> new_multiplicity;
        history_container.get_multiplicity(new_multiplicity);

        // 3. Resampling: Particles are drawn with replacement proportional
        // to their importance weight. This step is necessary since only a
        // finite number of particles is used to approximate a (continuous)
        // distribution. Furthermore, resampling allows us to apply a
        // particle filter in situations in which the target distribution
        // differs from the proposal. ***After resampling, all the particles
        // have the same weight.***

        // normalize weights before resampling
        float total_mass = 0.0;
        for( int i = 0; i < int(new_particles.size()); ++i )
            total_mass += new_particles[i].weight_ * new_multiplicity[i];
        assert(total_mass > 0);

        for( int i = 0; i < int(new_particles.size()); ++i )
            new_particles[i].weight_ /= total_mass;

        // resampling
        std::vector<int> indices;
        if( do_resampling_ ) {
            if( do_stochastic_universal_sampling_ )
                PF_t<PTYPE, BASE>::stochastic_universal_sampling(new_particles, new_multiplicity, nparticles_, indices);
            else
                PF_t<PTYPE, BASE>::stochastic_sampling(new_particles, new_multiplicity, nparticles_, indices);
        } else {
            indices.reserve(nparticles_);
            for( int i = 0; i < int(new_particles.size()); ++i ) {
                for( int j = 0; j < new_multiplicity[i]; ++j )
                    indices.push_back(i);
            }
        }
        assert(int(indices.size()) == nparticles_);

        // clean current filter
        for( int i = 0; i < int(particles_.size()); ++i )
            delete particles_[i].p_;
        particles_.clear();

        // set new particles as the particles in the updated filter
        multiplicity_.clear();
        std::map<int, int> index_map;
        for( int i = 0; i < int(indices.size()); ++i ) {
            int index = indices[i];
            std::map<int, int>::const_iterator it = index_map.find(index);
            if( it == index_map.end() ) {
                index_map.insert(std::make_pair(index, particles_.size()));
                particles_.push_back(new_particles[index]);
                multiplicity_.push_back(1);
            } else {
                assert(it->second < int(multiplicity_.size()));
                ++multiplicity_[it->second];
            }
        }

#ifdef USE_MPI 
        // processes were lauched to calculate marginals, now collect marginals
        assert(mpi_base_t::mpi_ != 0);
        assert(mpi_base_t::mpi_->nworkers_ >= 1);
        for( int i = 0; i < int(particles_.size()); ++i ) {
            PTYPE *p = particles_[i].p_;
            p->mpi_update_marginals(mpi_base_t::mpi_, particles_[i].wid_);
        }   
#endif



#if 0
        // 1. re-sampling according to importance weights
        std::vector<int> indices;
        if( do_resampling_ ) {
            if( do_stochastic_universal_sampling_ )
                PF_t<PTYPE, BASE>::stochastic_universal_sampling(nparticles_, indices);
            else
                PF_t<PTYPE, BASE>::stochastic_sampling(nparticles_, indices);
        } else {
            indices.reserve(nparticles_);
            for( int i = 0; i < int(particles_.size()); ++i ) {
                for( int j = 0; j < multiplicity_[i]; ++j )
                    indices.push_back(i);
            }
        }
        assert(int(indices.size()) == nparticles_);

#ifdef USE_MPI
        // prepare load balancing
        std::vector<std::vector<int> > mpi_budget = mpi_fixed_budget_;
        std::vector<int> mpi_load(mpi_budget.size(), 0);
        ++mpi_load[mpi_machine_for_master_];
#endif

        // 2. sampling next particles
        null_history_container_t history_container;
        //standard_history_container_t history_container;
        std::vector<particle_t> new_particles;
        for( int i = 0; i < int(indices.size()); ++i ) {
            int index = indices[i];
            const PTYPE &p = *particles_[index].p_;
            float weight = particles_[index].weight_;
            int wid = -1;

#ifdef      USE_MPI
            // do load balancing: find wid for machine with lowest load
            int best_mid = -1;
            for( int mid = 0; mid < int(mpi_budget.size()); ++mid ) {
                if( !mpi_budget[mid].empty() && ((best_mid == -1) || (mpi_load[best_mid] > mpi_load[mid])) )
                    best_mid = mid;
            }
            assert(best_mid >= 0);
            wid = mpi_budget[best_mid].back();
            mpi_budget[best_mid].pop_back();
            ++mpi_load[best_mid];
            //std::cout << "best: mid=" << best_mid << ", wid=" << wid << ", load=" << mpi_load[best_mid] - 1 << std::endl;
#endif

            PTYPE *np = sample_from_pi(p, last_action, obs, history_container, wid);

            if( !history_container.contains(np->history()) ) {
                history_container.insert(np->history());
            } else {
                history_container.increase_multiplicity(np->history());
                delete np;
                continue;
            }

            float new_weight = weight * importance_weight(*np, p, last_action, obs);
            new_particles.push_back(particle_t(new_weight, np, wid));
        }

        // set multiplicity and normalize
        int nparticles = 0;
        float total_mass = 0.0;
        multiplicity_ = std::vector<int>(new_particles.size(), 0);
        for( int i = 0; i < int(new_particles.size()); ++i ) {
            //assert(history_container.contains(new_particles[i].p_->history()));
            multiplicity_[i] = history_container.multiplicity(new_particles[i].p_->history());
            total_mass += new_particles[i].weight_ * multiplicity_[i];
            nparticles += multiplicity_[i];
        }
        assert(nparticles_ == nparticles);

        for( int i = 0; i < int(new_particles.size()); ++i ) {
            if( total_mass == 0 )
                new_particles[i].weight_ = 1.0 / float(new_particles.size());
            else
                new_particles[i].weight_ /= total_mass;
        }

        // replace old particles by new particles
        for( int i = 0; i < int(particles_.size()); ++i )
            delete particles_[i].p_;
        particles_ = std::move(new_particles);

        execution_.push_back(std::make_pair(last_action, obs));

#ifdef USE_MPI 
        // processes were lauched to calculate marginals, now collect marginals
        assert(mpi_base_t::mpi_ != 0);
        assert(mpi_base_t::mpi_->nworkers_ >= 1);
        for( int i = 0; i < int(particles_.size()); ++i ) {
            PTYPE *p = particles_[i].p_;
            p->mpi_update_marginals(mpi_base_t::mpi_, particles_[i].wid_);
        }   
#endif
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
            float weight = particles_[i].weight_;
            const PTYPE &p = *particles_[i].p_;
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

    PTYPE* sample_from_pi(const PTYPE &p, int last_action, int obs, const history_container_t &history_container, int wid = -1) const {
        PTYPE *np = new PTYPE(p);
        sample_from_pi(*np, p, last_action, obs, history_container, wid);
        return np;
    }

    virtual std::string id() const {
        std::string id_str;
        id_str = std::string("PF(type=SIR,nparticles=") + std::to_string((long long)nparticles_)
          //+ ",sus=" + std::to_string(do_stochastic_universal_sampling_)
          + ")";
        return id_str;
    }
};

#endif

