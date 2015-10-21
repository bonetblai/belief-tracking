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
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "mpi.h"
#include <dai/alldai.h>

struct mpi_raw_t {
  private:
    int *send_tags_;
    int *recv_tags_;

  public:
    int nworkers_;
    int worker_id_;
    int name_len_;
    char processor_name_[MPI_MAX_PROCESSOR_NAME];

    enum { MPI_MASTER_WORKER = 0 };

    mpi_raw_t(int argc, const char **argv) {
        // initialize MPI and obtain number of workers and ID
        MPI_Init(&argc, const_cast<char ***>(&argv));
        MPI_Comm_size(MPI_COMM_WORLD, &nworkers_);
        MPI_Comm_rank(MPI_COMM_WORLD, &worker_id_);
        MPI_Get_processor_name(processor_name_, &name_len_);

        if( worker_id_ == MPI_MASTER_WORKER ) {
            std::cout << "MPI: master has started in " << processor_name_
                      << ": pid=" << getpid()
                      << std::endl;
        } else {
            std::cout << "MPI: worker " << worker_id_
                      << "/" << nworkers_ - 1
                      << " has started in " << processor_name_
                      << ": pid=" << getpid()
                      << std::endl;
        }

        // initialize MPI local data structures
        send_tags_ = new int[1 + nworkers_];
        recv_tags_ = new int[1 + nworkers_];
        bzero(send_tags_, (1 + nworkers_) * sizeof(int));
        bzero(recv_tags_, (1 + nworkers_) * sizeof(int));
    }
    ~mpi_raw_t() {
        delete[] send_tags_;
        delete[] recv_tags_;
        MPI_Finalize();
    }

    // low-level input/output
    void raw_send(const void *buffer, int count, MPI_Datatype type, int wid) {
        MPI_Send((void*)buffer, count, type, wid, ++send_tags_[wid], MPI_COMM_WORLD);
    }
    void raw_recv(void *buffer, int count, MPI_Datatype type, int wid = MPI_MASTER_WORKER) {
        MPI_Status status;
        MPI_Recv(buffer, count, type, wid, ++recv_tags_[wid], MPI_COMM_WORLD, &status);
        assert(status.MPI_SOURCE == wid);
        assert(status.MPI_TAG == recv_tags_[wid]);
    }

    // basic input/output
    void send_int(int datum, int wid) {
        raw_send(&datum, 1, MPI_INT, wid);
    }
    int recv_int(int wid = MPI_MASTER_WORKER) {
        int data = -1;
        raw_recv(&data, 1, MPI_INT, wid);
        return data;
    }
};

struct mpi_t : mpi_raw_t {
    std::vector<int> factor_offset_;
    int total_size_;

    float *io_buffer_;

    mpi_t(int argc, const char **argv)
      : mpi_raw_t(argc, argv), total_size_(0), io_buffer_(0) { }
    ~mpi_t() {
         delete[] io_buffer_;
    }

    void initialize_buffers(const std::vector<dai::Factor> &factors) {
        factor_offset_ = std::vector<int>(1 + factors.size(), 0);
        for( int i = 1; i <= int(factors.size()); ++i )
            factor_offset_[i] = factor_offset_[i-1] + factors[i-1].nrStates();
        total_size_ = factor_offset_.back();
        delete[] io_buffer_;
        io_buffer_ = new float[total_size_];
        //std::cout << "[wid=" << worker_id_ << "] initialize buffers: factors.sz=" << factors.size() << ", totalsz=" << total_size_ << std::endl;
    }

    // send/recv indices
    void send_indices(const std::vector<int> &indices, int wid) {
        send_int(int(indices.size()), wid);
        int *buffer = new int[indices.size()];
        for( int i = 0; i < int(indices.size()); ++i )
            buffer[i] = indices[i];
        raw_send(buffer, indices.size(), MPI_INT, wid);
        delete[] buffer;
    }
    void recv_indices(std::vector<int> &indices, int wid = MPI_MASTER_WORKER) {
        int n = recv_int(wid);
        indices = std::vector<int>(n, 0);
        raw_recv(&indices[0], n, MPI_INT, wid);
    }

    // send/recv factor parametrization
    void send_parametrization(const dai::Factor &factor, int wid) {
        send_int(int(factor.nrStates()), wid);
        float *buffer = new float[factor.nrStates()];
        for( int j = 0; j < int(factor.nrStates()); ++j )
            buffer[j] = factor[j];
        raw_send(buffer, int(factor.nrStates()), MPI_FLOAT, wid);
        delete[] buffer;
    }
    void recv_parametrization(dai::Factor &factor, int wid = MPI_MASTER_WORKER) {
        int nrstates = recv_int(wid);
        assert(nrstates == int(factor.nrStates()));
        float *buffer = new float[nrstates];
        raw_recv(buffer, nrstates, MPI_FLOAT, wid);
        for( int j = 0; j < nrstates; ++j )
            factor.set(j, buffer[j]);
        delete[] buffer;
    }

    // send/recv factor metadata
    void send_metadata(const dai::Factor &factor, int wid) {
        send_int(int(factor.vars().size()), wid);
        int *buffer = new int[factor.vars().size()];
        int j = 0;
        for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it, ++j ) {
            buffer[j] = it->label();
        }
        raw_send(buffer, int(factor.vars().size()), MPI_INT, wid);
        delete[] buffer;
    }
    void recv_metadata(dai::Factor &factor, const std::vector<dai::Var> &variables, int wid = MPI_MASTER_WORKER) {
        int nvars = recv_int(wid);
        int *buffer = new int[nvars];
        raw_recv(buffer, nvars, MPI_INT, wid);
        dai::VarSet vars;
        for( int varid = 0; varid < nvars; ++varid )
            vars |= variables[buffer[varid]];
        delete[] buffer;
        factor = dai::Factor(vars, (double)0);
    }

    // send/recv parametrizations for multiple factors
    void send_parametrizations(const std::vector<dai::Factor> &factors, int wid) {
        send_int(int(factors.size()), wid);
        for( int fid = 0; fid < int(factors.size()); ++fid )
            send_parametrization(factors[fid], wid);
    }
    void recv_parametrizations(std::vector<dai::Factor> &factors, int wid = MPI_MASTER_WORKER) {
        int fid = recv_int(wid);
        assert((fid >= 0) && (fid < int(factors.size())));
        recv_parametrization(factors[fid], wid);
    }

    // send/recv all factor parametrizations
    void send_all_parametrizations(const std::vector<dai::Factor> &factors, int wid) {
        assert(factor_offset_.size() == 1 + factors.size());
        for( int fid = 0; fid < int(factors.size()); ++fid ) {
            for( int j = 0; j < int(factors[fid].nrStates()); ++j ) {
                assert(factor_offset_[fid] + j < total_size_);
                io_buffer_[factor_offset_[fid] + j] = factors[fid][j];
            }
        }
        raw_send(io_buffer_, total_size_, MPI_FLOAT, wid);
    }
    void recv_all_parametrizations(std::vector<dai::Factor> &factors, int wid = MPI_MASTER_WORKER) {
        //std::cout << "[wid=" << worker_id_ << "] factor_offset_.sz=" << factor_offset_.size() << ", 1+factors.sz=" << 1 + factors.size() << std::endl;
        assert(factor_offset_.size() == 1 + factors.size());
        raw_recv(io_buffer_, total_size_, MPI_FLOAT, wid);
        for( int fid = 0; fid < int(factors.size()); ++fid ) {
            for( int j = 0; j < int(factors[fid].nrStates()); ++j ) {
                assert(factor_offset_[fid] + j < total_size_);
                factors[fid].set(j, io_buffer_[factor_offset_[fid] + j]);
            }
        }
    }

    // send/recv complete factor
    void send_factor(const dai::Factor &factor, int wid) {
        send_metadata(factor, wid);
        send_parametrization(factor, wid);
    }
    void recv_factor(dai::Factor &factor, const std::vector<dai::Var> &variables, int wid = MPI_MASTER_WORKER) {
        recv_metadata(factor, variables, wid);
        recv_parametrization(factor, wid);
    }
    dai::Factor recv_factor(const std::vector<dai::Var> &variables, int wid = MPI_MASTER_WORKER) {
        dai::Factor factor;
        recv_factor(factor, variables, wid);
        return factor;
    }

    // send/recv multiple complete factor
    void send_factors(const std::vector<dai::Factor> &factors, int wid) {
        send_int(int(factors.size()), wid);
        for( int fid = 0; fid < int(factors.size()); ++fid )
            send_factor(factors[fid], wid);
    }
    void recv_factors(std::vector<dai::Factor> &factors, const std::vector<dai::Var> &variables, int wid = MPI_MASTER_WORKER) {
        int nfactors = recv_int(wid);
        factors.clear();
        factors.reserve(nfactors);
        for( int i = 0; i < nfactors; ++i )
            factors.push_back(recv_factor(variables, wid));
    }

    // send/recv multiple complete factor by indices
    void send_factors(const std::vector<dai::Factor> &factors, const std::vector<int> &indices, int wid) {
        for( int i = 0; i < int(indices.size()); ++i )
            send_factor(factors[indices[i]], wid);
    }
    void recv_factors(std::vector<dai::Factor> &factors, const std::vector<dai::Var> &variables, const std::vector<int> &indices, int wid = MPI_MASTER_WORKER) {
        for( int i = 0; i < int(indices.size()); ++i )
            recv_factor(factors[indices[i]], variables, wid);
    }

    // send/recv variable
    void send_variable(const dai::Var &var, int wid) {
        int buffer[2];
        buffer[0] = int(var.label());
        buffer[1] = int(var.states());
        raw_send(buffer, 2, MPI_INT, wid);
    }
    void recv_variable(dai::Var &var, int wid = MPI_MASTER_WORKER) {
        int buffer[2];
        raw_recv(buffer, 2, MPI_INT, wid);
        var = dai::Var(buffer[0], buffer[1]);
    }
    dai::Var recv_variable(int wid = MPI_MASTER_WORKER) {
        dai::Var var;
        recv_variable(var, wid);
        return var;
    }

    // send/recv multiple variables
    void send_variables(const std::vector<dai::Var> &variables, int wid) {
        send_int(int(variables.size()), wid);
        for( int i = 0; i < int(variables.size()); ++i )
            send_variable(variables[i], wid);
    }
    void recv_variables(std::vector<dai::Var> &variables, int wid = MPI_MASTER_WORKER) {
        int nvars = recv_int(wid);
        variables.clear();
        variables.reserve(nvars);
        for( int i = 0; i < int(nvars); ++i )
            variables.push_back(recv_variable(wid));
    }

    // send/recv marginals (in recv, assume marginals contains proper factors, only reading parametrization)
    void send_marginals(const std::vector<dai::Factor> &marginals, int wid) {
        int size = 0;
        for( int i = 0; i < int(marginals.size()); ++i )
            size += int(marginals[i].nrStates());

        send_int(size, wid);
        float *buffer = new float[size];
        for( int i = 0, off = 0; i < int(marginals.size()); ++i ) {
            const dai::Factor &marginal = marginals[i];
            for( int j = 0; j < int(marginal.nrStates()); ++j )
                buffer[off++] = marginal[j];
        }
        raw_send(buffer, size, MPI_FLOAT, wid);
        delete[] buffer;
    }
    void recv_marginals(std::vector<dai::Factor> &marginals, int wid = MPI_MASTER_WORKER) {
        int size = recv_int(wid);
        float *buffer = new float[size];
        raw_recv(buffer, size, MPI_FLOAT, wid);
        for( int i = 0, off = 0; i < int(marginals.size()); ++i ) {
            dai::Factor &marginal = marginals[i];
            for( int j = 0; j < int(marginal.nrStates()); ++j )
                marginal.set(j, buffer[off++]);
        }
        delete[] buffer;
    }

    // send/recv commands
    void send_command(int command, int wid) {
        send_int(command, wid);
    }
    int recv_command(int wid = MPI_MASTER_WORKER) {
        int command = recv_int(wid);
        return command;
    }
};

#endif

