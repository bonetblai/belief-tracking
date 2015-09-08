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

#ifndef PPCBT2_H
#define PPCBT2_H

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


template <typename PTYPE, typename BASE> struct ppcbt2_t : public PF_t<PTYPE, BASE> {
    using tracking_t<BASE>::base_;
    using PF_t<PTYPE, BASE>::nparticles_;
    using PF_t<PTYPE, BASE>::particles_;
    using PF_t<PTYPE, BASE>::marginals_on_vars_;

    std::string algorithm_;
    dai::PropertySet opts_;

    ppcbt2_t(const std::string &name, const BASE &base, int nparticles)
      : PF_t<PTYPE, BASE>(name, base, nparticles) {
        create_particles();
    }
    virtual ~ppcbt2_t() { }

    void set_algorithm_and_options(const std::string &algorithm, const dai::PropertySet &opts) {
        algorithm_ = algorithm;
        opts_ = opts;
    }

    void create_particles() {
        particles_.reserve(nparticles_);
        for( int i = 0; i < nparticles_; ++i )
            particles_.push_back(std::make_pair(1, particle_pcbt_t(&base_)));
    }

    virtual void initialize(int initial_loc) {
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].first = 1 / float(nparticles_);
            particles_[i].second.reset_factors(initial_loc);
        }
    }

    virtual void update(int last_action, int obs) {
        std::vector<int> indices;
        PF_t<PTYPE, BASE>::stochastic_universal_sampling(nparticles_, indices);
        assert(indices.size() == size_t(nparticles_));

        float total_mass = 0.0;
        std::vector<std::pair<float, particle_pcbt_t> > new_particles;
        new_particles.reserve(nparticles_);
        for( int i = 0; i < nparticles_; ++i ) {
            int index = indices[i];
            const particle_pcbt_t &p = particles_[index].second;
            int new_loc = p.new_loc(last_action, obs);

            float new_weight = 0;
            for( int label = 0; label < base_.nlabels_; ++label ) { // marginalize over possible labels at current loc
                new_weight += base_.obs_probability(obs, label, last_action) * p.probability(label, new_loc);
            }

            particle_pcbt_t np = p;
            np.update(last_action, obs, new_loc);
            new_particles.push_back(std::make_pair(new_weight, np));
            total_mass += new_weight;
        }
        for( int i = 0; i < nparticles_; ++i )
            new_particles[i].first /= total_mass;
        particles_ = new_particles;
    }

    virtual void calculate_marginals() {
        for( size_t i = 0; i < particles_.size(); ++i )
            particles_[i].second.calculate_marginals(algorithm_, opts_);
    }

    virtual void get_marginal(int var, dai::Factor &marginal) const {
        marginal = dai::Factor(dai::VarSet(dai::Var(var, var < base_.nloc_ ? base_.nlabels_ : base_.nloc_)), 0.0);
        dai::Factor pmarginal(marginal);
        for( int i = 0; i < nparticles_; ++i ) {
            particles_[i].second.get_marginal(var, pmarginal);
            pmarginal *= particles_[i].first;
            marginal += pmarginal;
        }
    }
};

#endif

