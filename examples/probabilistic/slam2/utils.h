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

inline int sample_from_distribution(int n, const float *cdf) {
#if 1 // do sampling in log(n) time
    assert(n > 0);
    if( n == 1 ) return 0;

    float p = drand48();
    int lower = 0;
    int upper = n;
    int mid = (lower + upper) >> 1;
    float lcdf = mid == 0 ? 0 : cdf[mid - 1];

# if 0
    std::cout << "sample_from_distribution:" << std::endl << "    cdf:";
    float prev = 0;
    for( int i = 0; i < n; ++i ) {
        std::cout << " " << cdf[i] - prev;
        prev = cdf[i];
    }
    std::cout << std::endl
              << "    p=" << p << std::endl
              << "    initial: lower=" << lower << ", mid=" << mid << ", upper=" << upper << ", lcdf=" << lcdf
              << std::endl;
# endif

    while( !(lcdf <= p) || !(p < cdf[mid]) ) {
        assert(lower < upper);
        assert((lower <= mid) && (mid < upper));
        if( lcdf > p ) {
            upper = mid;
        } else {
            if( !(p >= cdf[mid]) ) {
                std::cout << std::endl;
                std::cout << "cdf:";
                for( int i = 0; i < n; ++i ) std::cout << " " << cdf[i];
                std::cout << std::endl;
                std::cout << "p=" << p << ", mid=" << mid << std::endl;
            }
            assert(p >= cdf[mid]);
            lower = 1 + mid;
        }
        mid = (lower + upper) >> 1;
        lcdf = mid == 0 ? 0 : cdf[mid - 1];
# if 0
        std::cout << "    update: lower=" << lower << ", mid=" << mid << ", upper=" << upper << ", lcdf=" << lcdf
                  << std::endl;
# endif
    }
    assert((lcdf <= p) && (p < cdf[mid]));
    //std::cout << "    return: mid=" << mid << std::endl;
    return mid;
#else
    float p = drand48();
    for( int i = 0; i < n; ++i )
        if( p < cdf[i] ) return i;
    return n - 1;
#endif
}

inline void stochastic_sampling(int n, const float *cdf, int k, std::vector<int> &indices) {
    indices.clear();
    indices.reserve(k);
    for( int i = 0; i < k; ++i ) {
        int index = Utils::sample_from_distribution(n, cdf);
        indices.push_back(index);
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

};

#endif

