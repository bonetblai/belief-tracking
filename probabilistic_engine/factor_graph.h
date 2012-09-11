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


#ifndef FACTOR_GRAPH_H
#define FACTOR_GRAPH_H

#include <iostream>
#include <iomanip>
#include <cassert>
#include <vector>

//#define DEBUG

namespace BP {

template<typename T> inline void print_vector(std::ostream &os, int size, const T *vector, int prec = 2) {
    int old_prec = os.precision();
    os << std::setprecision(prec) << "[";
    for( int i = 0; i < size; ++i ) {
        T entry = vector[i];
        if( entry != 0 ) os << i << ":" << entry << ",";
    }
    os << "]" << std::setprecision(old_prec);
}

template<typename T> inline void print_implicit_vector(std::ostream &os, int size, int extra, T (*vector_fptr)(int extra, int i), int prec = 2) {
    int old_prec = os.precision();
    os << std::setprecision(prec) << "[";
    for( int i = 0; i < size; ++i ) {
        T entry = vector_fptr(extra, i);
        if( entry != 0 ) os << i << ":" << entry << ",";
    }
    os << "]" << std::setprecision(old_prec);
}


// Forward references
template<typename T> struct abstract_factor_t;


// Factor graph
class factor_graph_t {
  public:
    typedef std::pair<int, int> edge_t;
    typedef std::vector<int> node_list_t;
    typedef std::vector<edge_t> edge_list_t;

  protected:
    int nvars_;
    int nfactors_;
    int nedges_;

    std::vector<edge_list_t> edges_at_var_;
    std::vector<edge_list_t> edges_at_factor_;

  public:
    factor_graph_t(int nvars = 0, int nfactors = 0) {
        create_empty_graph(nvars, nfactors);
    }
    factor_graph_t(const factor_graph_t &graph)
      : nvars_(graph.nvars_),
        nfactors_(graph.nfactors_),
        nedges_(graph.nedges_),
        edges_at_var_(graph.edges_at_var_),
        edges_at_factor_(graph.edges_at_factor_) {
    }
    factor_graph_t(factor_graph_t &&graph)
      : nvars_(graph.nvars_),
        nfactors_(graph.nfactors_),
        nedges_(graph.nedges_),
        edges_at_var_(std::move(graph.edges_at_var_)),
        edges_at_factor_(std::move(graph.edges_at_factor_)) {
    }
    ~factor_graph_t() { }

    void create_empty_graph(int nvars, int nfactors) {
        nvars_ = nvars;
        nfactors_ = nfactors;
        nedges_ = 0;
        edges_at_var_ = std::vector<edge_list_t>(nvars_);
        edges_at_factor_ = std::vector<edge_list_t>(nfactors_);
    }

    void clear() {
        nedges_ = 0;
        edges_at_var_ = std::vector<edge_list_t>(nvars_);
        edges_at_factor_ = std::vector<edge_list_t>(nfactors_);
    }

    int nvars() const { return nvars_; }
    int nfactors() const { return nfactors_; }
    int nedges() const { return nedges_; }

    const edge_list_t& edges_at_variable(int vid) const {
        return edges_at_var_[vid];
    }
    const edge_list_t& edges_at_factor(int fid) const {
        return edges_at_factor_[fid];
    }

    void reserve_edge_list_at_var(int vid, int reservation) {
        edges_at_var_[vid].reserve(reservation);
    }
    void reserve_edge_list_at_factor(int fid, int reservation) {
        edges_at_factor_[fid].reserve(reservation);
    }

    void add_edge(int vid, int fid) {
        ++nedges_;
        edges_at_var_[vid].push_back(std::make_pair(vid, fid));
        edges_at_factor_[fid].push_back(std::make_pair(fid, vid));
    }
};


// Variables
template<typename T> struct variable_t {
    int id_;
    int domsz_;
    int nfactors_;
    abstract_factor_t<T> **factors_;
    int *fid_to_fix_;

    int msgsz_;
    T *stored_messages_;
    mutable T *next_messages_;

    T *marginal_;

    variable_t(int id, int domsz, int nfactors)
      : id_(id), domsz_(domsz), nfactors_(nfactors),
        factors_(new abstract_factor_t<T>*[nfactors_]),
        fid_to_fix_(0), msgsz_(0),
        stored_messages_(0), next_messages_(0),
        marginal_(new T[domsz_]) {
        memset(factors_, 0, nfactors_ * sizeof(abstract_factor_t<T>*));
        memset(marginal_, 0, domsz_ * sizeof(T));
    }
    virtual ~variable_t() {
        delete[] factors_;
        delete[] fid_to_fix_;
        delete[] stored_messages_;
        delete[] next_messages_;
        delete[] marginal_;
    }

    void set_factor(int fix, abstract_factor_t<T> *factor) {
        assert(fix < nfactors_);
        factors_[fix] = factor;
    }

    void fill_messages_by_default() {
        for( int i = 0; i < msgsz_; ++i )
            stored_messages_[i] = 1;
    }

    const T* message_to_factor_by_fid(int fid) const {
        return &stored_messages_[fid_to_fix_[fid] * domsz_];
    }
    const T* message_to_factor_by_fix(int fix) const {
        return &stored_messages_[fix * domsz_];
    }

    const T* marginal() const { return marginal_; }

    void initialize(int total_nfactors);
    void read_inbound_messages();
    void compute_marginal();

    void read_message_from_factor(int fix, const T *msg_from_fix);

    void calculate_outbound_messages() {
        read_inbound_messages();
    }

    void update_outbound_messages() {
        memcpy(stored_messages_, next_messages_, msgsz_ * sizeof(T));
        for( int fix = 0; fix < nfactors_; ++fix ) {
            bool need_to_copy = false;
            for( int i = 0; i < domsz_; ++i ) {
                if( ((next_messages_[fix * domsz_ + i] == 0) && (stored_messages_[fix * domsz_ + i] > 0)) ||
                    ((next_messages_[fix * domsz_ + i] > 0) && (stored_messages_[fix * domsz_ + i] == 0)) ) {
                    need_to_copy = true;
                    break;
                }
            }
            if( need_to_copy ) {
                //std::cout << "propagating message from vid=" << id_ << " to fix=" << fix << std::endl;
                memcpy(&stored_messages_[fix * domsz_], &next_messages_[fix * domsz_], domsz_ * sizeof(T));
            }
        }
    }

    bool consistent_marginal() const {
        T total_mass_in_marginal = 0;
        for( int i = 0; i < domsz_; ++i ) {
            if( (marginal_[i] < 0) || (marginal_[i] > 1) ) return false;
            total_mass_in_marginal += marginal_[i];
        }
        return total_mass_in_marginal == 1;
    }

    void print_marginal(std::ostream &os) const {
        print_vector(os, domsz_, marginal_);
    }

    void print_message_to_factor(std::ostream &os, int fid) const {
        int fix = fid_to_fix_[fid];
        os << "[vid=" << id_ << "] "
           << "msg to fid=" << fid << ": ";
        print_vector(os, domsz_, &stored_messages_[fix * domsz_]);
        os << std::endl;
    }
    void print_messages_to_factors(std::ostream &os) const {
        for( int fix = 0; fix < nfactors_; ++fix ) {
            print_message_to_factor(os, factors_[fix]->id_);
        }
    }
    void print_message_from_factor(std::ostream &os, int fid) const {
        int fix = fid_to_fix_[fid];
        os << "[vid=" << id_ << "] "
           << "msg from fid=" << fid << ": ";
        print_vector(os, domsz_, factors_[fix]->message_to_variable_by_vid(id_));
        os << std::endl;
    }
    void print_messages_from_factors(std::ostream &os) const {
        for( int fix = 0; fix < nfactors_; ++fix ) {
            print_message_from_factor(os, factors_[fix]->id_);
        }
    }
};


// Factors
template<typename T> struct abstract_factor_t {
    int id_;
    int nvars_;
    variable_t<T> **variables_;
    int *vid_to_vix_;
    int factorsz_;

    int msgsz_;
    int *offsets_;
    T *stored_messages_;
    mutable T *next_messages_;

    abstract_factor_t(int id, int nvars)
      : id_(id), nvars_(nvars),
        variables_(new variable_t<T>*[nvars_]),
        vid_to_vix_(0), factorsz_(0), msgsz_(0),
        offsets_(0), stored_messages_(0), next_messages_(0) {
        memset(variables_, 0, nvars_ * sizeof(variable_t<T>*));
    }
    virtual ~abstract_factor_t() {
        delete[] variables_;
        delete[] vid_to_vix_;
        delete[] offsets_;
        delete[] stored_messages_;
        delete[] next_messages_;
    }

    int factorsz() const { return factorsz_; }
    void set_variable(int vix, variable_t<T> *variable) {
        assert(vix < nvars_);
        variables_[vix] = variable;
    }

    virtual void initialize(int total_nvars) = 0;
    virtual void copy_factor(T *workspace) const = 0;
    virtual void marginalize(const T *workspace, int vix, T *msg_to_vix) const = 0;
    virtual void read_message_from_variable(T *workspace, int vjx, const T *msg_from_vjx) const = 0;
    virtual void print(std::ostream &os) const = 0;

    void allocate_space(int total_nvars) {
        // allocate space for messages
        msgsz_ = 0;
        offsets_ = new int[nvars_];
        for( int vix = 0; vix < nvars_; ++vix ) {
            assert(variables_[vix] != 0);
            offsets_[vix] = msgsz_;
            msgsz_ += variables_[vix]->domsz_;
        }
        stored_messages_ = new T[msgsz_];
        next_messages_ = new T[msgsz_];

        // calculate vid->vix map
        delete[] vid_to_vix_;
        vid_to_vix_ = new int[total_nvars];
        for( int vix = 0; vix < nvars_; ++vix ) {
            assert(variables_[vix] != 0);
            vid_to_vix_[variables_[vix]->id_] = vix;
        }

        // calculate factorsz
        factorsz_ = 1;
        for( int vix = 0; vix < nvars_; ++vix )
            factorsz_ *= variables_[vix]->domsz_;
    }

    void fill_messages_by_default() {
        for( int i = 0; i < msgsz_; ++i )
            stored_messages_[i] = 1;
    }

    const T* message_to_variable_by_vid(int vid) const {
        return &stored_messages_[offsets_[vid_to_vix_[vid]]];
    }
    const T* message_to_variable_by_vix(int vix) const {
        return &stored_messages_[offsets_[vix]];
    }

    void read_inbound_messages(T *workspace) const {
        if( nvars_ == 1 ) {
            copy_factor(next_messages_); // a litte better than the code below
        } else {
            for( int vix = 0; vix < nvars_; ++vix ) {
                copy_factor(workspace);
                //if( vix == 0 ) std::cout << "msg[0]=" << workspace[0] << std::endl;
                for( int vjx = 0; vjx < nvars_; ++vjx ) {
                    if( vjx != vix ) {
                        read_message_from_variable(workspace, vjx, variables_[vjx]->message_to_factor_by_fid(id_));
                        //if( vix == 0 ) std::cout << "msg[0]=" << workspace[0] << std::endl;
                    }
                }
                marginalize(workspace, vix, &next_messages_[offsets_[vix]]);
            }
            //std::cout << "id=" << id_ << std::endl;
            //if( id_ == 14 ) print_messages(std::cout);
        }
    }

    void calculate_outbound_messages(T *workspace) const {
        read_inbound_messages(workspace);
    }

    void update_outbound_messages() {
        if( nvars_ == 1 ) {
          memcpy(stored_messages_, next_messages_, msgsz_ * sizeof(T));
          return;
        }

        for( int vix = 0; vix < nvars_; ++vix ) {
            int domsz = variables_[vix]->domsz_;
            bool need_to_copy = false;
            for( int i = 0; i < domsz; ++i ) {
                if( ((next_messages_[offsets_[vix] + i] == 0) && (stored_messages_[offsets_[vix] + i] > 0)) ||
                    ((next_messages_[offsets_[vix] + i] > 0) && (stored_messages_[offsets_[vix] + i] == 0)) ) {
                    need_to_copy = true;
                    break;
                }
            }
            if( need_to_copy ) {
                //std::cout << "propagating message from fid=" << id_ << " to vid=" << vix << std::endl;
                memcpy(&stored_messages_[offsets_[vix]], &next_messages_[offsets_[vix]], domsz * sizeof(T));
            }
        }
    }

    void print_message_to_variable(std::ostream &os, int vid) const {
        int vix = vid_to_vix_[vid];
        os << "[fid=" << id_ << "] "
           << "msg to vid=" << vid << ": ";
        print_vector(os, variables_[vix]->domsz_, &stored_messages_[offsets_[vix]]);
        os << std::endl;
    }
    void print_messages_to_factors(std::ostream &os) const {
        for( int vix = 0; vix < nvars_; ++vix ) {
            print_message_to_variable(os, variables_[vix]->id_);
        }
    }
    void print_message_from_variable(std::ostream &os, int vid) const {
        int vix = vid_to_vix_[vid];
        os << "[fid=" << id_ << "] "
           << "msg from vid=" << vid << ": ";
        print_vector(os, variables_[vix]->domsz_, variables_[vix]->message_to_factor_by_fid(id_));
        os << std::endl;
    }
    void print_messages_from_variables(std::ostream &os) const {
        for( int vix = 0; vix < nvars_; ++vix ) {
            print_message_from_variable(os, variables_[vix]->id_);
        }
    }
};


// Methods for variables (inline)

template<typename T> void variable_t<T>::initialize(int total_nfactors) {
    // allocate space for messages
    msgsz_ = nfactors_ * domsz_;
    delete[] stored_messages_;
    stored_messages_ = new T[msgsz_];
    delete[] next_messages_;
    next_messages_ = new T[msgsz_];

    // calculate fid->fix map
    delete[] fid_to_fix_;
    fid_to_fix_ = new int[total_nfactors];
    for( int fix = 0; fix < nfactors_; ++fix ) {
        assert(factors_[fix] != 0);
        fid_to_fix_[factors_[fix]->id_] = fix;
    }
}

template<typename T> void variable_t<T>::read_message_from_factor(int fix, const T *msg_from_fix) {
    for( int fjx = 0; fjx < nfactors_; ++fjx ) {
        if( fjx != fix ) {
            for( int i = 0; i < domsz_; ++i ) {
                next_messages_[fjx * domsz_ + i] *= msg_from_fix[i];
            }
        }
    }
}

template<typename T> void variable_t<T>::read_inbound_messages() {
    for( int i = 0; i < msgsz_; ++i )
        next_messages_[i] = 1;
    for( int fix = 0; fix < nfactors_; ++fix ) {
        read_message_from_factor(fix, factors_[fix]->message_to_variable_by_vid(id_));
    }

#if 1 // CHECK: I think this shouldn't be normalized
    for( int fix = 0; fix < nfactors_; ++fix ) {
        T total_mass_in_msg_to_fix = 0;
        for( int i = 0; i < domsz_; ++i )
            total_mass_in_msg_to_fix += next_messages_[fix * domsz_ + i];
        //std::cout << "total mass for fix=" << fix << " is " << total_mass_msg_to_fix << std::endl;
        assert(total_mass_in_msg_to_fix > 0);
        if( total_mass_in_msg_to_fix > 0 ) {
            assert(total_mass_in_msg_to_fix > 0);
            for( int i = 0; i < domsz_; ++i )
                next_messages_[fix * domsz_ + i] /= total_mass_in_msg_to_fix;
        }
    }
#endif
}
 
template<typename T> void variable_t<T>::compute_marginal() {
    T total_mass_in_marginal = 0;
    for( int i = 0; i < domsz_; ++i ) {
        marginal_[i] = 1;
        for( int fix = 0; (marginal_[i] > 0) && (fix < nfactors_); ++fix ) {
            const T *msg_from_fix = factors_[fix]->message_to_variable_by_vid(id_);
            marginal_[i] *= msg_from_fix[i];
        }
        total_mass_in_marginal += marginal_[i];
    }
    assert(total_mass_in_marginal > 0);
    if( total_mass_in_marginal > 0 ) { // CHECK THIS!
        assert(total_mass_in_marginal > 0);
        for( int i = 0; i < domsz_; ++i )
            marginal_[i] /= total_mass_in_marginal;
    }
}

}; // end of namespace BP

#undef DEBUG

#endif

