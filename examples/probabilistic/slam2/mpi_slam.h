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

#ifndef MPI_SLAM_H
#define MPI_SLAM_H

#include <cassert>
#include <iostream>
#include <vector>

#include "mpi_api.h"

struct mpi_slam_t : public mpi_t {
    mpi_slam_t(int argc, const char **argv) : mpi_t(argc, argv) { }
    ~mpi_slam_t() { }

    enum { INITIALIZE, CALCULATE, READ, FINALIZE };

    void initialize_worker(const std::vector<dai::Var> &variables, const std::vector<dai::Factor> &factors, int tid) {
        send_command(INITIALIZE, tid);
        send_variables(variables, tid);
        send_factors(factors, tid);
    }

    void calculate_marginals(const std::vector<dai::Factor> &factors, const std::vector<int> &indices, int tid) {
        send_command(CALCULATE, tid);
        send_indices(indices, tid);
        send_factors(factors, indices, tid);
    }

    void read_marginals_from_worker(std::vector<dai::Factor> &marginals, int tid) {
        send_command(READ, tid);
        recv_marginals(marginals, tid);
    }

    void finalize_worker(int tid) {
        send_command(FINALIZE, tid);
    }

    void finalize_workers() {
        for( int tid = 1; tid < ntasks_; ++tid )
            finalize_worker(tid);
    }
};

#endif

