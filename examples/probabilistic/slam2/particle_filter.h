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

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <cassert>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <stdlib.h>

#include "utils.h"

#include <dai/alldai.h>


// Generic Particle Filter
template <typename PTYPE, typename BASE> struct PF_t : public tracking_t<BASE> {

    // particle type
    struct particle_t {
        float weight_;
        PTYPE *p_;
        int wid_;
        particle_t(float weight, PTYPE *p, int wid = -1)
          : weight_(weight), p_(p), wid_(wid) {
        }
        void print(std::ostream &os) const { p_->print(os); }
    };

    int nparticles_;
    std::vector<particle_t> particles_;
    std::vector<int> multiplicity_;
    std::vector<dai::Factor> marginals_on_vars_;

    PF_t(const std::string &name, const BASE &base, int nparticles)
      : tracking_t<BASE>(name, base), nparticles_(nparticles) {
    }
    virtual ~PF_t() { }

    virtual void print(std::ostream &os) const {
        for( int i = 0; i < int(particles_.size()); ++i ) {
            const particle_t &p = particles_[i];
            os << "particle: index=" << i
               << ", weight=" << p.weight_
               << ", multiplicity=" << multiplicity_[i]
               << ": "
               << std::flush;
            p.print(os);
            os << std::endl;
        }
    }

    void clear_particles() {
        for( int i = 0; i < int(particles_.size()); ++i )
            delete particles_[i].p_;
        particles_.clear();
    }

    void stochastic_sampling(const std::vector<particle_t> &particles,
                             const std::vector<int> &multiplicity,
                             int k,
                             std::vector<int> &indices) const {
        assert(particles.size() == multiplicity.size());
        assert(!particles.empty());
        std::vector<float> cdf(particles.size(), 0);
        cdf[0] = particles[0].weight_ * multiplicity[0];
        for( int i = 1; i < int(particles.size()); ++i )
            cdf[i] = cdf[i - 1] + particles[i].weight_ * multiplicity[i];
        for( int i = 1; i < int(particles.size()); ++i )
            cdf[i] /= cdf.back();
        Utils::stochastic_sampling(cdf.size(), &cdf[0], k, indices);
    }
    void stochastic_sampling(int k, std::vector<int> &indices) const {
        stochastic_sampling(particles_, multiplicity_, k, indices);
    }

    void stochastic_universal_sampling(const std::vector<particle_t> &particles,
                                       const std::vector<int> &multiplicity,
                                       int k,
                                       std::vector<int> &indices) const {
        assert(particles.size() == multiplicity.size());
        assert(!particles.empty());
        std::vector<float> cdf(particles.size(), 0);
        cdf[0] = particles[0].weight_ * multiplicity[0];
        for( int i = 1; i < int(particles.size()); ++i )
            cdf[i] = cdf[i - 1] + particles[i].weight_ * multiplicity[i];
        for( int i = 1; i < int(particles.size()); ++i )
            cdf[i] /= cdf.back();
        Utils::stochastic_universal_sampling(cdf.size(), &cdf[0], k, indices);
    }
    void stochastic_universal_sampling(int k, std::vector<int> &indices) const {
        stochastic_universal_sampling(particles_, multiplicity_, k, indices);
    }

    int sample_from_distribution(int n, const float *cdf) const {
        return Utils::sample_from_distribution(n, cdf);
    }
};

#endif

