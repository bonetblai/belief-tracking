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

#ifndef BELIEF_TRACKING_H
#define BELIEF_TRACKING_H

#include <iostream>
#include <vector>

//#define DEBUG

namespace BeliefTracking {

class valuation_t : public std::vector<int> {
  public:
    valuation_t(int nvars = 0, int default_value = 0) : std::vector<int>(nvars, default_value) { }
    void print(std::ostream &os) const {
        os << "[";
        for( int k = 0, ksz = size(); k < ksz; ++k ) {
            os << "v" << k << "=" << (*this)[k];
            if( k < ksz - 1 ) os << ",";
        }
        os << "]";
    }
};

class event_t : public std::vector<std::pair<int, int> > {
  public:
    void print(std::ostream &os) const {
        os << "E(";
        for( int k = 0, ksz = size(); k < ksz; ++k ) {
            os << "v" << (*this)[k].first << "=" << (*this)[k].second;
            if( k < ksz - 1 ) os << ",";
        }
        os << ")";
    }
};

class belief_tracking_t {
  public:
    typedef void (belief_tracking_t::*det_progress_func_t)(int action, valuation_t &valuation);
    typedef float (belief_tracking_t::*filter_func_t)(int obs, int action, const valuation_t &valuation);

  public:
    belief_tracking_t() { }
    virtual ~belief_tracking_t() { }
    virtual void progress_and_filter(int action, int obs, det_progress_func_t progress, filter_func_t filter) = 0;
    virtual void print(std::ostream &os) const = 0;
};

}; // end of namespace BeliefTracking

std::ostream& operator<<(std::ostream &os, const BeliefTracking::valuation_t &valuation) {
    valuation.print(os);
    return os;
}

std::ostream& operator<<(std::ostream &os, const BeliefTracking::event_t &event) {
    event.print(os);
    return os;
}

#undef DEBUG

#endif

