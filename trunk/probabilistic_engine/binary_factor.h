/*
 *  Copyright (C) 2012 Universidad Simon Bolivar
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


#ifndef BINARY_FACTOR_H
#define BINARY_FACTOR_H

#include "factor_graph.h"

#include <iostream>
#include <cassert>

//#define DEBUG

namespace BP {

template<typename T> struct abstract_binary_factor_t : public abstract_factor_t<T> {
    abstract_binary_factor_t(int id) : abstract_factor_t<T>(id, 2) { }
    virtual ~abstract_binary_factor_t() { }

    virtual void marginalize(const T *workspace, int vix, T *msg_to_vix) const {
        assert((vix == 0) || (vix == 1));
        int vix_domsz = abstract_factor_t<T>::variables_[vix]->domsz_;
        int vjx = 1 - vix;
        int vjx_domsz = abstract_factor_t<T>::variables_[vjx]->domsz_;
        int vjx_inc = vix == 0 ? 1 : vix_domsz;
        T total_mass_in_msg_to_vix = 0;
        for( int i = 0; i < vix_domsz; ++i ) {
            msg_to_vix[i] = 0;
            const T *fptr = vix == 0 ? &workspace[i * vjx_domsz] : &workspace[i];
            for( int j = 0; j < vjx_domsz; ++j ) {
                msg_to_vix[i] += *fptr;
                fptr += vjx_inc;
            }
            total_mass_in_msg_to_vix += msg_to_vix[i];
        }
        assert(total_mass_in_msg_to_vix > 0);

#if 1 // CHECK: do we need to normalize?
        if( total_mass_in_msg_to_vix > 0 ) {
            assert(total_mass_in_msg_to_vix > 0);
            for( int i = 0; i < vix_domsz; ++i ) {
                msg_to_vix[i] /= total_mass_in_msg_to_vix;
            }
        }
#endif
    }

    virtual void read_message_from_variable(T *workspace, int vjx, const T *msg_from_vjx) const {
        assert((vjx == 0) || (vjx == 1));
        int vjx_domsz = abstract_factor_t<T>::variables_[vjx]->domsz_;
        int vix = 1 - vjx;
        int vix_domsz = abstract_factor_t<T>::variables_[vix]->domsz_;
        int vix_inc = vjx == 0 ? 1 : vjx_domsz;
        T total_mass_in_msg_from_vjx = 0;
        for( int j = 0; j < vjx_domsz; ++j ) {
            T value = msg_from_vjx[j];
            T *fptr = vjx == 0 ? &workspace[j * vix_domsz] : &workspace[j];
            for( int i = 0; i < vix_domsz; ++i ) {
                *fptr *= value;
                fptr += vix_inc;
            }
            total_mass_in_msg_from_vjx += value;
        }
        assert(total_mass_in_msg_from_vjx > 0);
    }
};

template<typename T> struct explicit_binary_factor_t : public abstract_binary_factor_t<T> {
    T *factor_;

    explicit_binary_factor_t(int id) : abstract_binary_factor_t<T>(id) { }
    virtual ~explicit_binary_factor_t() { }

    void set_factor(const T *factor) {
        memcpy(factor_, factor, abstract_factor_t<T>::factorsz_ * sizeof(T));
    }

    virtual void initialize(int total_nvars) {
        abstract_factor_t<T>::allocate_space(total_nvars);
        factor_ = new T[abstract_factor_t<T>::factorsz_];
    }
    virtual void copy_factor(T *workspace) const {
        memcpy(workspace, factor_, abstract_factor_t<T>::factorsz_ * sizeof(T));
    }
    virtual void print(std::ostream &os) const {
        print_vector(os, abstract_factor_t<T>::factorsz_, factor_);
    }
};

template<typename T> struct implicit_binary_factor_t : public abstract_binary_factor_t<T> {
    T (*factor_entry_fptr_)(int fid, int i);
    void (*factor_vector_fptr_)(int fid, T *factor);

    implicit_binary_factor_t(int id,
                             T (*factor_entry_fptr)(int fid, int i) = 0,
                             void (*factor_vector_fptr)(int fid, T *factor) = 0)
      : abstract_binary_factor_t<T>(id),
        factor_entry_fptr_(factor_entry_fptr),
        factor_vector_fptr_(factor_vector_fptr) { }
    virtual ~implicit_binary_factor_t() { }

    void set_function_pointers(T (*factor_entry_fptr)(int fid, int i),
                               void (*factor_vector_fptr)(int fid, T *factor)) {
        factor_entry_fptr_ = factor_entry_fptr;
        factor_vector_fptr_ = factor_vector_fptr;
    }

    virtual void initialize(int total_nvars) {
        abstract_factor_t<T>::allocate_space(total_nvars);
    }
    virtual void copy_factor(T *workspace) const {
        factor_vector_fptr_(abstract_factor_t<T>::id_, workspace);
    }
    virtual void print(std::ostream &os) const {
        print_implicit_vector(os, abstract_factor_t<T>::factorsz_, abstract_factor_t<T>::id_, factor_entry_fptr_);
    }
};

}; // end of namespace BP

#undef DEBUG

#endif

