/*
 *  Copyright (C) 2014 Universidad Simon Bolivar
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

#ifndef PROBABILISTIC_BELIEF_TRACKING_H
#define PROBABILISTIC_BELIEF_TRACKING_H

#include <iostream>
#include "belief_tracking.h"

//#define DEBUG

namespace ProbabilisticBeliefTracking {

class probabilistic_belief_tracking_t : public BeliefTracking::belief_tracking_t {
  public:
    probabilistic_belief_tracking_t() { }
    virtual ~probabilistic_belief_tracking_t() { }
    virtual float probability(const BeliefTracking::event_t &event) const = 0;
    virtual void print(std::ostream &os) const = 0;
};

}; // end of namespace ProbabilisticBeliefTracking

#undef DEBUG

#endif

