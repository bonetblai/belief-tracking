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

#ifndef OPTIMAL_SIR2_H
#define OPTIMAL_SIR2_H

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


// SIR filter with proposal distribution given by optimal choice (intractable)
template <typename PTYPE, typename BASE> struct optimal_SIR2_t : public SIR2_t<PTYPE, BASE> {
    using PF_t<PTYPE, BASE>::sample_from_distribution;

    int nobs_; // nlabels
    int nstates_;
    float **cdf_;

    optimal_SIR2_t(const std::string &name, const BASE &base, int nparticles)
      : SIR2_t<PTYPE, BASE>(name, base, nparticles) {
#if 0
        nstates_ = nloc_;
        for( int loc = 0; loc < nloc_; ++loc )
            nstates_ *= nlabels_;
        calculate_cdf_for_pi();
#endif
    }
    virtual ~optimal_SIR2_t() {
#if 0
        for( int i = 0; i < nloc_ * nlabels_ * 4; ++i )
            delete[] cdf_[i];
        delete[] cdf_;
#endif
    }

    void calculate_cdf_for_pi() {
#if 0
        cdf_ = new float*[nstates_ * nlabels_ * 4];
        for( int i = 0; i < nstates_ * nlabels_ * 4; ++i ) {
            cdf_[i] = new float[nstates_];
            base_particle_t p;
            int state_index = i % nstates_;
            int obs = (i / nstates_) % nlabels_;
            int action = (i / nstates_) / nlabels_;
            decode(state_index, p);
            for( int state_index2 = 0; state_index2 < nstates_; ++state_index2 ) {
                base_particle_t p2;
                decode(state_index2, p2);
                cdf_[i][state_index2] = state_index2 > 0 ? cdf_[i][state_index2 - 1] : 0;
                cdf_[i][state_index2] += pi(p2, p, action, obs);
            }
            //cerr << "cdf[" << i << "/" << nstates_ * nlabels_ * 4 << "][" << nstates_ - 1 << "]="
            //     << setprecision(9) << cdf_[i][nstates_ - 1] << endl;
            assert(fabs(cdf_[i][nstates_ - 1] - 1.0) < .0001);
        }
#endif
    }
    const float* pi_cdf(int state_index, int last_action, int obs) const {
        return cdf_[(last_action * nobs_ + obs) * nstates_ + state_index];
    }

    virtual void sample_from_pi(PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        const float *dist = pi_cdf(p.encode(), last_action, obs);
        int state_index = sample_from_distribution(nstates_, dist);
        np.decode(state_index);
    }

    float pi(const PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        // np has probability:
        //   P(np|p,last_action,obs) = P(np,obs|p,last_action) / P(obs|p,last_action)
        //                           = P(obs|np,p,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        //                           = P(obs|np,last_action) * P(np|p,last_action) / P(obs|p,last_action)
        return p.pi(np, p, last_action, obs);
    }

    virtual float importance_weight(const PTYPE &np, const PTYPE &p, int last_action, int obs) const {
        // weight = P(obs|np,last_action) * P(np|p,last_action) / P(np|p,last_action,obs)
        //        = P(obs|p,last_action) [see above derivation in pi(..)]
        //        = SUM P(np,obs|p,last_action)
        //        = SUM P(obs|np,p,last_action) P(np|p,last_action)
        //        = SUM P(obs|np,last_action) P(np|p,last_action)
        return p.importance_weight(np, p, last_action, obs);
    }
};

#endif

