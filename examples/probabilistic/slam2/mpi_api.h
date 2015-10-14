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

#ifndef MPI_API_H
#define MPI_API_H

#include <cassert>
#include <iostream>
#include <string>
#include <vector>

#include "mpi.h"
#include <dai/alldai.h>

struct mpi_t {
    int ntasks_;
    int taskid_;
    int name_len_;
    char processor_name_[MPI_MAX_PROCESSOR_NAME];

    int *send_tags_;
    int *recv_tags_;

    enum { MPI_MASTER_TASK = 0 };

    mpi_t(int argc, const char **argv) {
        // initialize MPI and obtain number of tasks and ID
        MPI_Init(&argc, const_cast<char ***>(&argv));
        MPI_Comm_size(MPI_COMM_WORLD, &ntasks_);
        MPI_Comm_rank(MPI_COMM_WORLD, &taskid_);
        MPI_Get_processor_name(processor_name_, &name_len_);
        std::cout << "MPI task " << taskid_
                  << "/" << ntasks_
                  << " has started in " << processor_name_
                  << ": pid=" << getpid()
                  << std::endl;

        // initialize MPI local data structures
        send_tags_ = new int[1 + ntasks_];
        recv_tags_ = new int[1 + ntasks_];
        bzero(send_tags_, (1 + ntasks_) * sizeof(int));
        bzero(recv_tags_, (1 + ntasks_) * sizeof(int));
    }
    ~mpi_t() {
        delete[] send_tags_;
        delete[] recv_tags_;
        MPI_Finalize();
    }


    // low-level input/output
    void raw_send(const void *buffer, int count, MPI_Datatype type, int tid) {
        MPI_Send((void*)buffer, count, type, tid, ++send_tags_[tid], MPI_COMM_WORLD);
    }
    void raw_recv(void *buffer, int count, MPI_Datatype type, int tid = MPI_MASTER_TASK) {
        MPI_Status status;
        MPI_Recv(buffer, count, type, tid, ++recv_tags_[tid], MPI_COMM_WORLD, &status);
        assert(status.MPI_SOURCE == tid);
        assert(status.MPI_TAG == recv_tags_[tid]);
    }

    // basic input/output
    void send_int(int datum, int tid) {
        raw_send(&datum, 1, MPI_INT, tid);
    }
    int recv_int(int tid = MPI_MASTER_TASK) {
        int data = -1;
        raw_recv(&data, 1, MPI_INT, tid);
        return data;
    }

    // send/recv indices
    void send_indices(const std::vector<int> &indices, int tid) {
        send_int(int(indices.size()), tid);
        int *buffer = new int[indices.size()];
        for( int i = 0; i < int(indices.size()); ++i )
            buffer[i] = indices[i];
        raw_send(buffer, indices.size(), MPI_INT, tid);
        delete[] buffer;
    }
    void recv_indices(std::vector<int> &indices, int tid = MPI_MASTER_TASK) {
        int n = recv_int(tid);
        indices = std::vector<int>(n, 0);
        raw_recv(&indices[0], n, MPI_INT, tid);
    }

    // send/recv factor parametrization
    void send_parametrization(const dai::Factor &factor, int tid) {
        send_int(int(factor.nrStates()), tid);
        float *buffer = new float[factor.nrStates()];
        for( int j = 0; j < int(factor.nrStates()); ++j )
            buffer[j] = factor[j];
        raw_send(buffer, int(factor.nrStates()), MPI_FLOAT, tid);
        delete[] buffer;
    }
    void recv_parametrization(dai::Factor &factor, int tid = MPI_MASTER_TASK) {
        int nrstates = recv_int(tid);
        assert(nrstates == int(factor.nrStates()));
        float *buffer = new float[nrstates];
        raw_recv(buffer, nrstates, MPI_FLOAT, tid);
        for( int j = 0; j < nrstates; ++j )
            factor.set(j, buffer[j]);
        delete[] buffer;
    }

    // send/recv factor metadata
    void send_metadata(const dai::Factor &factor, int tid) {
        send_int(int(factor.vars().size()), tid);
        int *buffer = new int[factor.vars().size()];
        int j = 0;
        for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it, ++j ) {
            buffer[j] = it->label();
        }
        raw_send(buffer, int(factor.vars().size()), MPI_INT, tid);
        delete[] buffer;
    }
    void recv_metadata(dai::Factor &factor, const std::vector<dai::Var> &variables, int tid = MPI_MASTER_TASK) {
        int nvars = recv_int(tid);
        int *buffer = new int[nvars];
        raw_recv(buffer, nvars, MPI_INT, tid);
        dai::VarSet vars;
        for( int varid = 0; varid < nvars; ++varid )
            vars |= variables[buffer[varid]];
        delete[] buffer;
        factor = dai::Factor(vars, (double)0);
    }

    // send/recv parametrizations for multiple factors
    void send_parametrizations(const std::vector<dai::Factor> &factors, int tid) {
        send_int(int(factors.size()), tid);
        for( int fid = 0; fid < int(factors.size()); ++fid )
            send_parametrization(factors[fid], tid);
    }
    void recv_parametrizations(std::vector<dai::Factor> &factors, int tid = MPI_MASTER_TASK) {
        int fid = recv_int(tid);
        assert((fid >= 0) && (fid < int(factors.size())));
        recv_parametrization(factors[fid], tid);
    }

    // send/recv complete factor
    void send_factor(const dai::Factor &factor, int tid) {
        send_metadata(factor, tid);
        send_parametrization(factor, tid);
    }
    void recv_factor(dai::Factor &factor, const std::vector<dai::Var> &variables, int tid = MPI_MASTER_TASK) {
        recv_metadata(factor, variables, tid);
        recv_parametrization(factor, tid);
    }
    dai::Factor recv_factor(const std::vector<dai::Var> &variables, int tid = MPI_MASTER_TASK) {
        dai::Factor factor;
        recv_factor(factor, variables, tid);
        return factor;
    }

    // send/recv multiple complete factor
    void send_factors(const std::vector<dai::Factor> &factors, int tid) {
        send_int(int(factors.size()), tid);
        for( int fid = 0; fid < int(factors.size()); ++fid )
            send_factor(factors[fid], tid);
    }
    void recv_factors(std::vector<dai::Factor> &factors, const std::vector<dai::Var> &variables, int tid = MPI_MASTER_TASK) {
        int nfactors = recv_int(tid);
        factors.clear();
        factors.reserve(nfactors);
        for( int i = 0; i < nfactors; ++i )
            factors.push_back(recv_factor(variables, tid));
    }

    // send/recv multiple complete factor by indices
    void send_factors(const std::vector<dai::Factor> &factors, const std::vector<int> &indices, int tid) {
        for( int i = 0; i < int(indices.size()); ++i )
            send_factor(factors[indices[i]], tid);
    }
    void recv_factors(std::vector<dai::Factor> &factors, const std::vector<dai::Var> &variables, const std::vector<int> &indices, int tid = MPI_MASTER_TASK) {
        for( int i = 0; i < int(indices.size()); ++i )
            recv_factor(factors[indices[i]], variables, tid);
    }

    // send/recv variable
    void send_variable(const dai::Var &var, int tid) {
        int buffer[2];
        buffer[0] = int(var.label());
        buffer[1] = int(var.states());
        raw_send(buffer, 2, MPI_INT, tid);
    }
    void recv_variable(dai::Var &var, int tid = MPI_MASTER_TASK) {
        int buffer[2];
        raw_recv(buffer, 2, MPI_INT, tid);
        var = dai::Var(buffer[0], buffer[1]);
    }
    dai::Var recv_variable(int tid = MPI_MASTER_TASK) {
        dai::Var var;
        recv_variable(var, tid);
        return var;
    }

    // send/recv multiple variables
    void send_variables(const std::vector<dai::Var> &variables, int tid) {
        send_int(int(variables.size()), tid);
        for( int i = 0; i < int(variables.size()); ++i )
            send_variable(variables[i], tid);
    }
    void recv_variables(std::vector<dai::Var> &variables, int tid = MPI_MASTER_TASK) {
        int nvars = recv_int(tid);
        variables.clear();
        variables.reserve(nvars);
        for( int i = 0; i < int(nvars); ++i )
            variables.push_back(recv_variable(tid));
    }

    // send/recv marginals (in recv, assume marginals contains proper factors, only reading parametrization)
    void send_marginals(const std::vector<dai::Factor> &marginals, int tid) {
        int size = 0;
        for( int i = 0; i < int(marginals.size()); ++i )
            size += int(marginals[i].nrStates());

        send_int(size, tid);
        float *buffer = new float[size];
        for( int i = 0, off = 0; i < int(marginals.size()); ++i ) {
            const dai::Factor &marginal = marginals[i];
            for( int j = 0; j < int(marginal.nrStates()); ++j )
                buffer[off++] = marginal[j];
        }
        raw_send(buffer, size, MPI_FLOAT, tid);
        delete[] buffer;
    }
    void recv_marginals(std::vector<dai::Factor> &marginals, int tid = MPI_MASTER_TASK) {
        int size = recv_int(tid);
        float *buffer = new float[size];
        raw_recv(buffer, size, MPI_FLOAT, tid);
        for( int i = 0, off = 0; i < int(marginals.size()); ++i ) {
            dai::Factor &marginal = marginals[i];
            for( int j = 0; j < int(marginal.nrStates()); ++j )
                marginal.set(j, buffer[off++]);
        }
        delete[] buffer;
    }

    // send/recv commands
    void send_command(int command, int tid) {
        send_int(command, tid);
    }
    int recv_command(int tid = MPI_MASTER_TASK) {
        int command = recv_int(tid);
        return command;
    }
};

#endif

