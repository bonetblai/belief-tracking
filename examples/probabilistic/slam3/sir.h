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

//#define DEBUG

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

class standard_history_container_t : public history_container_t {
  protected:
    std::map<history_t, int> multiplicities_;
    std::map<history_t, int> indices_;

  public:
    virtual int size() const {
        assert(multiplicities_.size() == indices_.size());
        return multiplicities_.size();
    }
    virtual bool contains(const history_t &history) const {
        return multiplicities_.find(history) != multiplicities_.end();
    }
    virtual void insert(const history_t &history, int index) {
        multiplicities_.insert(std::make_pair(history, 1));
        indices_.insert(std::make_pair(history, index));
    }
    virtual int multiplicity(const history_t &history) const {
        return multiplicities_.find(history)->second;
    }
    virtual int index(const history_t &history) const {
        return indices_.find(history)->second;
    }
    virtual void increase_multiplicity(const history_t &history) {
        ++multiplicities_[history];
    }
    virtual void get_multiplicity(std::vector<int> &multiplicity) const {
        multiplicity = std::vector<int>(indices_.size(), 0);
        for( std::map<history_t, int>::const_iterator it = indices_.begin(); it != indices_.end(); ++it )
            multiplicity[it->second] = multiplicities_.find(it->first)->second;
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

    bool force_resampling_;
    bool do_stochastic_universal_sampling_;
    int num_sampling_attempts_;

    std::vector<std::pair<int, int> > execution_;

#ifdef USE_MPI
    // MPI load balancing
    static int mpi_machine_for_master_;
    static std::vector<std::vector<int> > mpi_fixed_budget_;
#endif

    SIR_t(const std::string &name, const BASE &base, const std::multimap<std::string, std::string> &parameters)
      : PF_t<PTYPE, BASE>(name, base),
        force_resampling_(false),
        do_stochastic_universal_sampling_(false),
        num_sampling_attempts_(1) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("nparticles");
        if( it != parameters.end() )
            PF_t<PTYPE, BASE>::set_nparticles(strtol(it->second.c_str(), 0, 0));
        it = parameters.find("force-resampling");
        if( it != parameters.end() )
            force_resampling_ = it->second == "true";
        it = parameters.find("sus");
        if( it != parameters.end() )
            do_stochastic_universal_sampling_ = it->second == "true";
        it = parameters.find("num-sampling-attempts");
        if( it != parameters.end() )
            num_sampling_attempts_ = strtol(it->second.c_str(), 0, 0);
    }
    virtual ~SIR_t() { /* who deletes particles? */ }

    virtual bool sample_from_pi(PTYPE &np,
                                const PTYPE &p,
                                int last_action,
                                int obs,
                                const history_container_t &history_container,
                                int wid) const = 0;
    virtual float importance_weight(const PTYPE &np,
                                    const PTYPE &p,
                                    int last_action,
                                    int obs) const = 0;

    virtual void clear() {
        PF_t<PTYPE, BASE>::clear_particles();
    }

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

    virtual void update(int last_action, int obs) {
#ifdef DEBUG
        std::cout << "# last-action=" << last_action << ", obs=" << obs << std::endl;
        std::cout << "# ***weights:";
        for( int i = 0; i < int(particles_.size()); ++i )
            std::cout << " " << particles_[i].weight_;
        std::cout << std::endl;
#endif

        // 1. Importance sampling: The next generation of particles is obtained
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

        std::vector<particle_t> new_particles;
        standard_history_container_t history_container;
        for( int i = 0; i < int(particles_.size()); ++i ) {
            const PTYPE &p = *particles_[i].p_;
            float weight = particles_[i].weight_;
            for( int j = 0; j < multiplicity_[i]; ++j ) {
                int wid = -1;

#ifdef USE_MPI
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
#ifdef DEBUG
                std::cout << "# best: mid=" << best_mid << ", wid=" << wid << ", load=" << mpi_load[best_mid] - 1 << std::endl;
#endif
#endif

                // attempt to sample from pi using particle p as base particle.
                // If MPI is used, we assume that failed sampling implies wid is not
                // used
                bool successful_attempt = false;
                for( int k = 0; (k < num_sampling_attempts_) && !successful_attempt; ++k ) {
                    // sample new particle using the proposal distribution \pi
                    PTYPE *np = sample_from_pi(p, last_action, obs, history_container, wid);
                    if( np != 0 ) {
                        // mark this attempts as successful
                        successful_attempt = true;

                        // update histories and multiplicities
                        if( !history_container.contains(np->history()) ) {
                            history_container.insert(np->history(), new_particles.size());
                        } else {
                            history_container.increase_multiplicity(np->history());
                            delete np;
                            continue;
                        }

                        // compute new importance weight using recursive formula and add new particle
                        float new_weight = weight * importance_weight(*np, p, last_action, obs);
                        new_particles.push_back(particle_t(new_weight, np, wid));
                    } else {
                        if( 1 + k == num_sampling_attempts_ ) std::cout << "(removing particle)" << std::flush;
                    }
                }
            }
        }
        assert(int(new_particles.size()) == history_container.size());
        std::vector<int> new_multiplicity;
        history_container.get_multiplicity(new_multiplicity);

        // normalize weights and decide whether to perform re-sampling
        float total_weight = 0.0;
        for( int i = 0; i < int(new_particles.size()); ++i )
            total_weight += new_particles[i].weight_ * new_multiplicity[i];
        assert(total_weight > 0);

        // normalize and compute N_eff criterion
        float Neff = 0;
        for( int i = 0; i < int(new_particles.size()); ++i ) {
            new_particles[i].weight_ /= total_weight;
            Neff += new_multiplicity[i] * new_particles[i].weight_ * new_particles[i].weight_;
        }
        Neff = 1 / Neff;

        // decide resampling
        float resampling_threshold = float(nparticles_) / 2;
        bool do_resampling = force_resampling_ || (Neff < resampling_threshold);

#ifdef DEBUG
        std::cout << "# N_eff=" << Neff << ", threshold=" << resampling_threshold << ", do-resampling=" << do_resampling << std::endl;
#endif

        // do resampling
        std::vector<int> indices;
        if( do_resampling ) {
            // 3. Resampling: Particles are drawn with replacement proportional
            // to their importance weight. This step is necessary since only a
            // finite number of particles is used to approximate a (continuous)
            // distribution. Furthermore, resampling allows us to apply a
            // particle filter in situations in which the target distribution
            // differs from the proposal.
            //
            // After resampling, all the particles have the same weight.
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
        assert(!do_resampling || (int(indices.size()) == nparticles_));

        // clean current filter
        PF_t<PTYPE, BASE>::clear_particles();

        // set new particles as the particles in the updated filter
        multiplicity_.clear();
        std::map<int, int> index_map;
        std::set<int> selected_indices;
        for( int i = 0; i < int(indices.size()); ++i ) {
            int index = indices[i];
            selected_indices.insert(index);
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

        // if resampling was performed, clean unsampled particles
        // and set all particle's weight to same value
        if( do_resampling ) {
            for( int i = 0; i < int(new_particles.size()); ++i ) {
                if( selected_indices.find(i) == selected_indices.end() )
                    delete new_particles[i].p_;
            }

            // calcualte number of particles in new filter
            int num_particles = 0;
            for( int i = 0; i < int(particles_.size()); ++i )
                num_particles += multiplicity_[i];

            // set weights
            float weight = 1 / float(num_particles);
            for( int i = 0; i < int(particles_.size()); ++i )
                particles_[i].weight_ = weight;

#ifdef DEBUG
            std::cout << "#         N_eff: " << Neff << std::endl;
            std::cout << "#     threshold: " << resampling_threshold << std::endl;
            std::cout << "#    resampling: " << do_resampling << std::endl;
            std::cout << "#multiplicities:";
            for( int i = 0; i < int(multiplicity_.size()); ++i )
                std::cout << " " << multiplicity_[i];
            std::cout << std::endl;
            std::cout << "#       weights:";
            for( int i = 0; i < int(particles_.size()); ++i )
                std::cout << " " << particles_[i].weight_;
            std::cout << std::endl;
#endif
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
        if( !sample_from_pi(*np, p, last_action, obs, history_container, wid) ) {
            delete np;
            return 0;
        } else {
            return np;
        }
    }

    virtual std::string id() const {
        std::string id_str;
        id_str = std::string("PF(type=SIR") +
          std::string(",ptype=") + PTYPE::type() +
          std::string(",nparticles=") + std::to_string((long long)nparticles_) +
          std::string(",force-resampling=") + (force_resampling_ ? "true" : "false") +
          std::string(",sus=") + (do_stochastic_universal_sampling_ ? "true" : "false") +
          std::string(",num-sampling-attempts=") + std::to_string((long long)num_sampling_attempts_) +
          ")";
        return id_str;
    }
};

#undef DEBUG

#endif

