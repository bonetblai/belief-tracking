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
    int nparticles_;
    std::vector<std::pair<float, PTYPE> > particles_;
    std::vector<dai::Factor> marginals_on_vars_;

    PF_t(const std::string &name, const BASE &base, int nparticles)
      : tracking_t<BASE>(name, base), nparticles_(nparticles) {
    }
    virtual ~PF_t() { }

    void stochastic_sampling(int k, std::vector<int> &indices) const {
        std::vector<float> cdf(nparticles_, 0);
        cdf[0] = particles_[0].first;
        for( int i = 1; i < nparticles_; ++i )
            cdf[i] = cdf[i - 1] + particles_[i].first;
        return Utils::stochastic_sampling(cdf.size(), &cdf[0], k, indices);
    }

    void stochastic_universal_sampling(int k, std::vector<int> &indices) const {
        std::vector<float> cdf(nparticles_, 0);
        cdf[0] = particles_[0].first;
        for( int i = 1; i < nparticles_; ++i )
            cdf[i] = cdf[i - 1] + particles_[i].first;
        return Utils::stochastic_universal_sampling(cdf.size(), &cdf[0], k, indices);
    }

    int sample_from_distribution(int n, const float *cdf) const {
        return Utils::sample_from_distribution(n, cdf);
    }
};

#endif

