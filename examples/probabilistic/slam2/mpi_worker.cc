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

#include <cassert>
#include <string>
#include <iostream>
#include <string.h>
#include <vector>
#include <stdlib.h>
#include <math.h>

#include <dai/alldai.h>

#include "inference.h"
#include "utils.h"

#include "mpi_slam.h"

using namespace std;


// static members
string inference_t::algorithm_;
string inference_t::options_;
dai::PropertySet inference_t::libdai_options_;
string inference_t::type_;
string inference_t::edbp_factors_fn_;
string inference_t::edbp_evid_fn_;
string inference_t::edbp_output_fn_;
int inference_t::edbp_max_iter_;

// global vars
inference_t g_inference;
vector<dai::Var> g_variables;
vector<dai::Factor> g_factors;
vector<int> g_indices_for_updated_factors;
vector<int> g_indices_for_all_factors;
vector<dai::Factor> g_marginals;
vector<vector<float> > g_test;
vector<vector<int> > g_edbp_factor_indices;
bool g_print_marginals = false;


void print_factor(ostream &os, int fid, const std::vector<dai::Factor> &factors, const std::string &name) {
    g_inference.print_factor(os, fid, factors, name);
}

void print_factors(ostream &os, const std::vector<dai::Factor> &factors, const std::string &name = "g_factors") {
    for( int fid = 0; fid < int(factors.size()); ++fid )
        print_factor(os, fid, factors, name);
}

void compute_edbp_factor_indices() {
    g_edbp_factor_indices = vector<vector<int> >(10);

    // define indices to compute
    vector<int> number_vars;
    number_vars.push_back(2);
    number_vars.push_back(3);
    number_vars.push_back(4);
    number_vars.push_back(5);
    number_vars.push_back(6);
    number_vars.push_back(9);

    for( int i = 0; i < int(number_vars.size()); ++i ) {
        int nvars = number_vars[i];
        g_edbp_factor_indices[nvars] = vector<int>(1 << nvars);

        dai::VarSet vars, r_vars;
        for( int var = 0; var < nvars; ++var ) {
            vars |= dai::Var(var, 2);
            r_vars |= dai::Var(nvars - var - 1, 2);
        }
        //cout << "vars=" << vars << ", r_vars=" << r_vars << endl;

        for( int value = 0; value < (1 << nvars); ++value ) {
            map<dai::Var, size_t> state = dai::calcState(vars, value);
            map<dai::Var, size_t> r_state;
            for( map<dai::Var, size_t>::const_iterator it = state.begin(); it != state.end(); ++it ) {
                int label = nvars - 1 - it->first.label();
                r_state[dai::Var(label, 2)] = it->second;
            }
            int r_index = dai::calcLinearState(r_vars, r_state);
            g_edbp_factor_indices[nvars][r_index] = value;
#if 0
            cout << "State of vars(index=" << value << "): " //<< state
                      << "; state of r_vars(index=" << r_index << "): " //<< dai::calcState(r_vars, r_index)
                      << endl;
#endif
        }
    }
}

int edbp_factor_index(int nvars, int index) {
    assert(nvars < int(g_edbp_factor_indices.size()));
    assert(index < int(g_edbp_factor_indices[nvars].size()));
    return g_edbp_factor_indices[nvars][index];
}

int edbp_factor_index(const dai::Factor &factor, int index) {
    return edbp_factor_index(factor.vars().size(), index);
}

void initialize_inference_engine(mpi_slam_t &mpi) {
    // receive variables and factors
    mpi.recv_variables(g_variables);
    mpi.recv_factors(g_factors, g_variables);
    mpi.initialize_buffers(g_factors);
    g_inference.create_and_initialize_algorithm(g_factors);

    // CHECK: should discriminate between BEL and MAR, act like BEL
    g_marginals = g_factors;

    // calculate indices for all factors
    g_indices_for_all_factors.reserve(g_factors.size());
    for( int fid = 0; fid < int(g_factors.size()); ++fid )
        g_indices_for_all_factors.push_back(fid);
}

void calculate(mpi_slam_t &mpi) {
    // receive parametrizations
    //std::cout << "[wid=" << mpi.worker_id_ << "] receive all parametrizations from master" << std::endl;
    mpi.recv_all_parametrizations(g_factors, mpi_slam_t::MPI_MASTER_WORKER);

    // calculate
    g_indices_for_updated_factors = g_indices_for_all_factors;
    g_inference.calculate_marginals(g_variables,
                                    g_indices_for_updated_factors,
                                    g_factors,
                                    g_marginals,
                                    edbp_factor_index,
                                    g_print_marginals);
}

void write_marginals(mpi_slam_t &mpi) {
    //std::cout << "[wid=" << mpi.worker_id_ << "] send marginals to master" << std::endl;
    mpi.send_marginals(g_marginals, mpi_slam_t::MPI_MASTER_WORKER);
}

bool execute_command(mpi_slam_t &mpi, int command) {
    if( command == mpi_slam_t::INITIALIZE ) {
        //std::cout << "[wid=" << mpi.worker_id_ << "] initializing engine..." << std::endl;
        initialize_inference_engine(mpi);
    } else if( command == mpi_slam_t::CALCULATE ) {
        //std::cout << "[wid=" << mpi.worker_id_ << "] calculating ..." << std::endl;
        calculate(mpi);
    } else if( command == mpi_slam_t::READ ) {
        //std::cout << "[wid=" << mpi.worker_id_ << "] writing marginals ..." << std::endl;
        write_marginals(mpi);
    } else if( command == mpi_slam_t::FINALIZE ) {
        //std::cout << "[wid=" << mpi.worker_id_ << "] finalizing engine..." << std::endl;
        return false;
    }
    return true;
}


void usage(ostream &os) {
    os << endl
       << "Usage: mpi_engine [{-i | --inference} <algorithm>]" << endl
       << "                  [--tmp-path <path>]" << endl
       << "                  [{-s | --seed} <seed>]" << endl
       << "                  [{-v | --verbose}]" << endl
       << "                  [{-? | --help}]" << endl
       << endl;
}

int main(int argc, const char **argv) {
    // initialize MPI and obtain number of worker and ID
    mpi_slam_t mpi(argc, argv);

    // parse arguments
    int seed = 0;
    bool verbose = false;
    string tmp_path = "";
    string inference_algorithm = "jt(updates=HUGIN)";

    --argc;
    ++argv;
    while( (argc > 0) && (**argv == '-') ) {
        if( !strcmp(argv[0], "-i") || !strcmp(argv[0], "--inference") ) {
            inference_algorithm = argv[1];
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "--tmp-path") ) {
            tmp_path = argv[1];
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-v") || !strcmp(argv[0], "--verbose") ) {
            verbose = true;
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-?") || !strcmp(argv[0], "--help") ) {
            usage(cerr);
            exit(-1);
        } else {
            cerr << "error: unexpected argument: " << argv[0] << endl;
            --argc;
            ++argv;
        }
    }

    // set seed using supplied seed and MPI rank
    seed += mpi.worker_id_;
    unsigned short seeds[3];
    seeds[0] = seeds[1] = seeds[2] = seed;
    seed48(seeds);
    srand48(seed);

    // initialize inference algorithm
    compute_edbp_factor_indices();
    inference_t::set_inference_algorithm(inference_algorithm, "BEL", tmp_path, verbose);

    // main execution loop
    bool continue_execution = true;
    while( continue_execution ) {
        int command = mpi.recv_command();
        continue_execution = execute_command(mpi, command);
    }
    return 0;
}

