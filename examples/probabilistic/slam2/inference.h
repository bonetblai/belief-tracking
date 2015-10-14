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

#ifndef INFERENCE_H
#define INFERENCE_H

#include <cassert>
#include <iostream>
#include <iomanip>
#include <map>
#include <string>
#include <vector>

#include <dai/alldai.h>

struct inference_t {
    static std::string algorithm_;
    static std::string options_;

    static dai::PropertySet libdai_options_;
    static std::string type_;

    static std::string edbp_factors_fn_;
    static std::string edbp_evid_fn_;
    static std::string edbp_output_fn_;
    static int edbp_max_iter_;

    // inference algorithm
    dai::InfAlg *inference_algorithm_;

    inference_t() : inference_algorithm_(0) { }
    inference_t(inference_t &&i) {
        inference_algorithm_ = i.inference_algorithm_;
        i.inference_algorithm_ = 0;
    }
    ~inference_t() {
        destroy_inference_algorithm();
    }

    const inference_t& operator=(const inference_t &i) {
        if( i.inference_algorithm_ != 0 )
            inference_algorithm_ = i.inference_algorithm_->clone();
        else
            inference_algorithm_ = 0;
        return *this;
    }

    bool operator==(const inference_t &i) const {
        return ((inference_algorithm_ == 0) && (i.inference_algorithm_ == 0)) ||
               ((inference_algorithm_ != 0) && (i.inference_algorithm_ != 0));
    }

    static void set_inference_algorithm(const std::string &algorithm_spec, const std::string &type, const std::string &tmp_path, bool verbose = true) {
        if( verbose ) {
            std::cout << "# inference algorithm set to '"
                      << algorithm_spec << "'"
                      << std::endl;
        }

        type_ = type;
        parse_inference_algorithm(algorithm_spec);

        // if edbp algorithm, create evidence file and set filenames
        if( algorithm_ == "edbp" ) {
            int pid = getpid();
            assert(tmp_path.empty() || (tmp_path.back() == '/'));
            edbp_factors_fn_ = tmp_path + "dummy" + std::to_string(pid) + ".factors";
            edbp_evid_fn_ = tmp_path + "dummy" + std::to_string(pid) + ".evid";
            edbp_output_fn_ = "/dev/null";
            system((std::string("echo 0 > ") + edbp_evid_fn_).c_str());
            if( libdai_options_.hasKey("maxiter") )
                edbp_max_iter_ = libdai_options_.getStringAs<size_t>("maxiter");
            else
                edbp_max_iter_ = 10;
        }
    }

    void clean_inference_algorithm() {
        if( algorithm_ == "edbp" ) {
            unlink(edbp_factors_fn_.c_str());
            unlink((edbp_factors_fn_ + "." + type_).c_str());
            unlink(edbp_evid_fn_.c_str());
            if( edbp_output_fn_ != "/dev/null" )
                unlink(edbp_output_fn_.c_str());
        }
    }

    static void parse_inference_algorithm(const std::string &algorithm_spec) {
        size_t first_par = algorithm_spec.find_first_of('(');
        size_t last_par = algorithm_spec.find_first_of(')');
        assert(first_par != std::string::npos);
        assert(last_par != std::string::npos);
        assert(last_par == algorithm_spec.size() - 1);

        algorithm_ = std::string(algorithm_spec, 0, first_par);
        options_ = std::string(algorithm_spec, first_par + 1, last_par - first_par - 1);

        // parameter types
        std::map<std::string, std::string> parameter_type;
        parameter_type["bbp_props"] = "string";
        parameter_type["bbp_cfn"] = "string";
        parameter_type["cavity"] = "string";
        parameter_type["choose"] = "string";
        parameter_type["clamp"] = "string";
        parameter_type["clusters"] = "string";
        parameter_type["damping"] = "double";
        parameter_type["doubleloop"] = "boolean";
        parameter_type["init"] = "string";
        parameter_type["inits"] = "string";
        parameter_type["logdomain"] = "boolean";
        parameter_type["loopdepth"] = "size_t";
        parameter_type["maxiter"] = "size_t";
        parameter_type["maxtime"] = "double";
        parameter_type["min_max_adj"] = "double";
        parameter_type["rec_tol"] = "double";
        parameter_type["recursion"] = "string";
        parameter_type["tol"] = "double";
        parameter_type["type"] = "string";
        parameter_type["updates"] = "string";
        parameter_type["verbose"] = "size_t";

        // set parameters
        std::vector<std::string> tokens = dai::tokenizeString(options_, false, ", ");
        for( int i = 0; i < int(tokens.size()); ++i ) {
            size_t equal_pos = tokens[i].find_first_of('=');
            assert(equal_pos != std::string::npos);
            std::string parameter(tokens[i], 0, equal_pos);
            std::string value(tokens[i], equal_pos + 1);

            // string paramenters
            if( parameter_type.find(parameter) == parameter_type.end() ) {
                std::cout << "warning: parameter '"
                          << parameter
                          << "' not recognized"
                          << std::endl;
            } else {
                const std::string &type = parameter_type[parameter];
                if( type == "string" ) {
                    libdai_options_ = libdai_options_(parameter, value);
                } else if( type == "double" ) {
                    libdai_options_ = libdai_options_(parameter, atof(value.c_str()));
                } else if( type == "boolean" ) {
                    libdai_options_ = libdai_options_(parameter, value == "true");
                } else if( type == "size_t" ) {
                    libdai_options_ = libdai_options_(parameter, size_t(atol(value.c_str())));
                } else {
                    std::cout << "warning: type '"
                              << type
                              << "' not supported"
                              << std::endl;
                    }
            }
        }
    }

    void create_and_initialize_algorithm(const std::vector<dai::Factor> &factors, bool verbose = false) {
        if( algorithm_ != "edbp" ) {
            dai::FactorGraph factor_graph(factors);

            if( algorithm_ == "jt" ) {
                inference_algorithm_ = new dai::JTree(factor_graph, libdai_options_);
            } else if( algorithm_ == "bp" ) {
                inference_algorithm_ = new dai::BP(factor_graph, libdai_options_);
            } else if( algorithm_ == "cbp" ) {
                inference_algorithm_ = new dai::CBP(factor_graph, libdai_options_);
            } else if( algorithm_ == "lc" ) {
                inference_algorithm_ = new dai::LC(factor_graph, libdai_options_);
            } else if( algorithm_ == "mr" ) {
                inference_algorithm_ = new dai::MR(factor_graph, libdai_options_);
            } else if( algorithm_ == "hak" ) {
                inference_algorithm_ = new dai::HAK(factor_graph, libdai_options_);
            } else {
                std::cout << "error: unrecognized inference algorithm '"
                          << algorithm_ << "'"
                          << std::endl;
                exit(-1);
            }

            if( inference_algorithm_ != 0 ) {
                if( verbose ) {
                    std::cout << "[algorithm="
                              << algorithm_
                              << inference_algorithm_->printProperties()
                              << "]"
                              << std::flush;
                }
                inference_algorithm_->init();
            }
        }
    }

    void destroy_inference_algorithm() {
        delete inference_algorithm_;
        inference_algorithm_ = 0;
    }

    void apply_inference_libdai(const std::vector<int> &indices_for_updated_factors, const std::vector<dai::Factor> &factors) const {
        assert(inference_algorithm_ != 0);
        assert(!indices_for_updated_factors.empty());
        dai::VarSet variables;
        for( int i = 0; i < int(indices_for_updated_factors.size()); ++i ) {
            int index = indices_for_updated_factors[i];
            inference_algorithm_->fg().setFactor(index, factors[index]);
            variables |= factors[index].vars();
        }
        inference_algorithm_->init(variables);
        inference_algorithm_->run();
    }

    void generate_input_file_edbp(const std::vector<dai::Var> &variables, const std::vector<dai::Factor> &factors) const {
        // create model in given file
        std::ofstream ofs(edbp_factors_fn_);

        ofs << "MARKOV" << std::endl
            << variables.size()
            << std::endl;

        for( int vid = 0; vid < int(variables.size()); ++vid ) {
            ofs << variables[vid].states()
                << (vid < int(variables.size()) - 1 ? " " : "");
        }
        ofs << std::endl;
        
        ofs << factors.size() << std::endl;
        for( int fid = 0; fid < int(factors.size()); ++fid ) {
            const dai::Factor &factor = factors[fid];
            ofs << factor.vars().size();
            for( dai::VarSet::const_reverse_iterator it = factor.vars().rbegin(); it != factor.vars().rend(); ++it ) {
                ofs << " " << it->label();
            }
            ofs << std::endl;
        }
        ofs << std::endl;

        for( int fid = 0; fid < int(factors.size()); ++fid ) {
            print_factor_edbp(ofs, fid, factors);
            ofs << std::endl;
        }
        ofs.close();
    }

    void apply_inference_edbp(const std::vector<dai::Var> &variables, const std::vector<dai::Factor> &factors) const {
        generate_input_file_edbp(variables, factors);

        // call edbp solver
        std::string edbp_cmd =
          std::string("~/software/edbp/solver") + " " + edbp_factors_fn_
          + " " + edbp_evid_fn_ + " " + std::to_string(edbp_max_iter_)
          + " 0 " + type_ + " 2>" + edbp_output_fn_;
        system(edbp_cmd.c_str());
    }

    void extract_marginals_from_inference_libdai(
        const std::vector<dai::Var> &variables,
        const std::vector<dai::Factor> &factors,
        std::vector<dai::Factor> &marginals,
        bool print_marginals = false) const {
        if( type_ == "BEL" ) {
            assert(marginals.size() == factors.size());
            for( int fid = 0; fid < int(factors.size()); ++fid ) {
                marginals[fid] = inference_algorithm_->belief(factors[fid].vars());
                if( print_marginals )
                    print_factor(std::cout, fid, marginals, "libdai::marginals");
            }
        } else {
            assert(marginals.size() == variables.size());
            for( int vid = 0; vid < int(variables.size()); ++vid ) {
                marginals[vid] = inference_algorithm_->belief(variables[vid]);
                if( print_marginals )
                    print_factor(std::cout, vid, marginals, "libdai::marginals");
            }
        }
    }

    void extract_marginals_from_inference_edbp(
        const std::vector<dai::Var> &variables,
        const std::vector<dai::Factor> &factors,
        std::vector<dai::Factor> &marginals,
        int edbp_factor_index(const dai::Factor&, int),
        bool print_marginals = false) const {

        assert((type_ == "MAR") || (edbp_factor_index != 0));
        std::ifstream ifs(edbp_factors_fn_ + "." + type_);

        std::string buff;
        ifs >> buff;
        assert(buff == type_);

        bool more_results = true;
        while( more_results ) {
            int nlines = 0;
            ifs >> nlines;
            assert(nlines == 1);

            if( type_ == "BEL" ) {
                int nfactors = 0;
                ifs >> nfactors;
                assert(nfactors == int(factors.size()));
                assert(nfactors == int(marginals.size()));
                for( int fid = 0; fid < nfactors; ++fid ) {
                    const dai::Factor &factor = factors[fid];
                    int nstates = 0;
                    ifs >> nstates;
                    assert(nstates == int(factor.nrStates()));
                    for( int j = 0; j < nstates; ++j ) {
                        float p = 0;
                        ifs >> p;
                        marginals[fid].set(edbp_factor_index(factor, j), p);
                    }
                }
            } else {
                int nvariables = 0;
                ifs >> nvariables;;
                assert(nvariables == int(variables.size()));
                assert(nvariables == int(marginals.size()));
                for( int vid = 0; vid < nvariables; ++vid ) {
                    const dai::Var &variable = variables[vid];
                    int nstates = 0;
                    ifs >> nstates;
                    assert(nstates == int(variable.states()));
                    for( int j = 0; j < nstates; ++j ) {
                        float p = 0;
                        ifs >> p;
                        marginals[vid].set(j, p);
                    }
                }
            }

            // check if there are more results
            std::string marker;
            ifs >> marker;
            more_results = marker == "-BEGIN-";
        }
        ifs.close();

        if( print_marginals ) {
            for( int fid = 0; fid < int(marginals.size()); ++fid ) {
                print_factor(std::cout, fid, marginals, "edbp::marginals");
            }
        }
    }

    void calculate_marginals(
        const std::vector<dai::Var> &variables,
        std::vector<int> &indices_for_updated_factors,
        const std::vector<dai::Factor> &factors,
        std::vector<dai::Factor> &marginals,
        int edbp_factor_index(const dai::Factor&, int) = 0,
        bool print_marginals = false) const {
        if( !indices_for_updated_factors.empty() ) {
            if( algorithm_ == "edbp" ) {
                apply_inference_edbp(variables, factors);
                extract_marginals_from_inference_edbp(variables, factors, marginals, edbp_factor_index, print_marginals);
            } else {
                apply_inference_libdai(indices_for_updated_factors, factors);
                extract_marginals_from_inference_libdai(variables, factors, marginals, print_marginals);
            }
            indices_for_updated_factors.clear();
        }
    }

    void print_factor(std::ostream &os, int fid, const std::vector<dai::Factor> &factors, const std::string &name) const {
        assert((fid >= 0) && (fid < int(factors.size())));
        const dai::Factor &factor = factors[fid];
        os << "variables[fid=" << fid << "]=";
        for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it )
            os << " " << *it;
        os << std::endl << name << "[fid=" << fid << "]=" << std::endl;
        for( int j = 0; j < int(factor.nrStates()); ++j ) {
            os << "   ";
            std::map<dai::Var, size_t> states = dai::calcState(factor.vars(), j);
            for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it )
                os << " " << std::setw(3) << *it << "=" << states[*it];
            os << ":  j=" << std::setw(3) << j << ",  nbits=" << "na" << ",  value=" << factor[j] << std::endl;
        }
    }

    void print_factor_edbp(std::ostream &os, int fid, const std::vector<dai::Factor> &factors) const {
        assert((fid >= 0) && (fid < int(factors.size())));
        const dai::Factor &factor = factors[fid];
        os << factor.nrStates() << std::endl;
        for( int j = 0; j < int(factor.nrStates()); ++j )
            os << " " << factor[j];
        os << std::endl;
    }
};

#endif // INFERENCE_H

