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

#ifndef UTILS_H
#define UTILS_H

#include <cassert>
#include <math.h>
#include <sys/resource.h>
#include <sys/time.h>

namespace Utils {

inline float read_time_in_seconds() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    float time = (float)r_usage.ru_utime.tv_sec + (float)r_usage.ru_utime.tv_usec / (float)1000000;
    getrusage(RUSAGE_CHILDREN, &r_usage);
    time += (float)r_usage.ru_utime.tv_sec + (float)r_usage.ru_utime.tv_usec / (float)1000000;
    return time;
}

inline void stochastic_sampling(int n, const float *cdf, int k, std::vector<int> &indices) {
    assert(cdf[n-1] != 0);
    indices.clear();
    indices.reserve(k);
    for( int j = 0; j < k; ++j ) {
        float u = drand48();
        if( u >= cdf[n - 1] ) {
            indices.push_back(n - 1);
        } else {
            for( int i = 0; i < n; ++i ) {
                if( u < cdf[i] ) {
                    indices.push_back(i);
                    break;
                }
            }
        }
    }
    assert(k == int(indices.size()));
}

inline void stochastic_universal_sampling(int n, const float *cdf, int k, std::vector<int> &indices) {
    indices.clear();
    indices.reserve(k);
    float u = drand48() / float(k);
    for( int i = 0, j = 0; j < k; ++j ) {
        while( (i < n) && (u > cdf[i]) ) ++i;
        indices.push_back(i == n ? n - 1 : i);
        u += 1.0 / float(k);
    }
    assert(k == int(indices.size()));
}

inline int sample_from_distribution(int n, const float *cdf) {
    float p = drand48();
    for( int i = 0; i < n; ++i )
        if( p <= cdf[i] ) return i;
    return n - 1;
}

};

#endif

