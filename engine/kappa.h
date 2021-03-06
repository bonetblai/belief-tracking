/*
 *  Copyright (C) 2016 Universidad Simon Bolivar
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

#ifndef KAPPA_H
#define KAPPA_H

#include <cassert>
#include <iostream>
#include <vector>
#include <math.h>

//#define DEBUG

class kappa_t {
  protected:
    static float epsilon_;
    static std::vector<float> powers_;

  public:
    static void initialize(float epsilon, int num_powers) {
        epsilon_ = epsilon;
        powers_ = std::vector<float>(num_powers);
        for( int i = 0; i < num_powers; ++i )
            powers_[i] = powf(epsilon_, i);
    }

    static float epsilon() { return epsilon_; }
    static float power(int i) {
        if( i == std::numeric_limits<int>::max() )
            return 0;
        else
            return i < int(powers_.size()) ? powers_[i] : powf(epsilon_, i);
    }
    static int kappa(float p) {
        if( p <= powers_.back() ) {
#ifdef DEBUG
            std::cout << "kappa: epsilon=" << epsilon_ << ", p=" << p << ", kappa=" << 1 + powers_.size() << " (limit)" << std::endl;
#endif
            return 1 + powers_.size();
        } else {
            int start = 0;
            int end = int(powers_.size()) - 1;
            while( start < end ) {
#ifdef DEBUG
                std::cout << "kappa: epsilon=" << epsilon_
                          << ", p=" << p
                          << ", start=" << start
                          << ", end=" << end
                          << ", pow[start]=" << powers_[start]
                          << ", pow[end]=" << powers_[end]
                          << std::endl;
#endif
                int mid = (start + end) / 2;
                assert(1 + mid < int(powers_.size()));
                if( (powers_[mid] >= p) && (p > powers_[1 + mid]) ) {
                    assert((p <= power(mid)) && (power(1 + mid) < p));
#ifdef DEBUG
                    std::cout << "kappa: epsilon=" << epsilon_ << ", p=" << p << ", kappa=" << mid << std::endl;
#endif
                    return mid;
                } else if( p > powers_[mid] ) {
                    end = mid;
                } else {
                    start = mid;
                }
            }
            assert(p == powers_[start]);
#ifdef DEBUG
            std::cout << "kappa: epsilon=" << epsilon_ << ", p=" << p << ", kappa=" << start << std::endl;
#endif
            return start;
        }
    }
};

#undef DEBUG

#endif

