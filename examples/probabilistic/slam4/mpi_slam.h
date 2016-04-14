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

#ifndef USE_MPI
struct mpi_slam_t { };
#else
#include "mpi_api.h"

struct mpi_slam_t : public mpi_t {
    mpi_slam_t(int argc, const char **argv) : mpi_t(argc, argv) { }
    ~mpi_slam_t() { }

    enum { INITIALIZE, CALCULATE, READ, FINALIZE };

    void initialize_worker(const std::vector<dai::Var> &variables, const std::vector<dai::Factor> &factors, int wid) {
        //std::cout << "[master] send INITIALIZE to wid=" << wid << std::endl;
        send_command(INITIALIZE, wid);
        //std::cout << "[master] send variables to wid=" << wid << std::endl;
        send_variables(variables, wid);
        //std::cout << "[master] send factors to wid=" << wid << std::endl;
        send_factors(factors, wid);
    }

    void calculate_marginals(const std::vector<dai::Factor> &factors, std::vector<int> &indices_for_updated_factors, int wid) {
        if( !indices_for_updated_factors.empty() ) {
            //std::cout << "[master] send CALCULATE to wid=" << wid << std::endl;
            send_command(CALCULATE, wid);
            //std::cout << "[master] send all parametrizations to wid=" << wid << std::endl;
            send_all_parametrizations(factors, wid);
            indices_for_updated_factors.clear();
        }
    }

    void read_marginals_from_worker(std::vector<dai::Factor> &marginals, int wid) {
        //std::cout << "[master] send READ to wid=" << wid << std::endl;
        send_command(READ, wid);
        //std::cout << "[master] read marginals from wid=" << wid << std::endl;
        recv_marginals(marginals, wid);
    }

    void finalize_worker(int wid) {
        send_command(FINALIZE, wid);
    }

    void finalize_workers() {
        for( int wid = 1; wid < nworkers_; ++wid )
            finalize_worker(wid);
    }
};
#endif

#endif

