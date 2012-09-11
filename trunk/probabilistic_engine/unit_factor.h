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


#ifndef UNIT_FACTOR_H
#define UNIT_FACTOR_H

#include "factor_graph.h"

#include <iostream>
#include <cassert>

//#define DEBUG

namespace BP {

template<typename T> struct abstract_unit_factor_t : public abstract_factor_t<T> {
    abstract_unit_factor_t(int id) : abstract_factor_t<T>(id, 1) { }
    virtual ~abstract_unit_factor_t() { }

    virtual void marginalize(const T *workspace, int vix, T *msg_to_vix) const {
        assert(vix == 0);
        memcpy(msg_to_vix, workspace, abstract_factor_t<T>::factorsz_ * sizeof(T));
    }

    virtual void read_message_from_variable(T *workspace, int vjx, const T *msg_from_vjx) const {
        assert(0); // the way messages are propagated implies this shouldn't be called
    }
};

template<typename T> struct explicit_unit_factor_t : public abstract_unit_factor_t<T> {
    T *factor_;

    explicit_unit_factor_t(int id) : abstract_unit_factor_t<T>(id) { }
    virtual ~explicit_unit_factor_t() { }

    void set_factor(const T *factor) {
        memcpy(factor_, factor, abstract_factor_t<T>::factorsz_ * sizeof(T));
    }

    virtual void initialize(int total_nvars) {
        abstract_factor_t<T>::allocate_space(total_nvars);
        assert(abstract_factor_t<T>::factorsz_ == abstract_factor_t<T>::variables_[0]->domsz_);
        factor_ = new T[abstract_factor_t<T>::factorsz_];
    }
    virtual void copy_factor(T *workspace) const {
        memcpy(workspace, factor_, abstract_factor_t<T>::factorsz_ * sizeof(T));
    }
    virtual void print(std::ostream &os) const {
        print_vector(os, abstract_factor_t<T>::factorsz_, factor_);
    }
};

template<typename T> struct implicit_unit_factor_t : public abstract_unit_factor_t<T> {
    T (*factor_entry_fptr_)(int fid, int i);
    void (*factor_vector_fptr_)(int fid, T *factor);

    implicit_unit_factor_t(int id,
                           T (*factor_entry_fptr)(int fid, int i) = 0,
                           void (*factor_vector_fptr)(int fid, T *factor) = 0)
      : abstract_unit_factor_t<T>(id),
        factor_entry_fptr_(factor_entry_fptr),
        factor_vector_fptr_(factor_vector_fptr) { }
    virtual ~implicit_unit_factor_t() { }

    void set_function_pointers(T (*factor_entry_fptr)(int fid, int i),
                               void (*factor_vector_fptr)(int fid, T *factor)) {
        factor_entry_fptr_ = factor_entry_fptr;
        factor_vector_fptr_ = factor_vector_fptr;
    }

    virtual void initialize(int total_nvars) {
        abstract_factor_t<T>::allocate_space(total_nvars);
        assert(abstract_factor_t<T>::factorsz_ == abstract_factor_t<T>::variables_[0]->domsz_);
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

