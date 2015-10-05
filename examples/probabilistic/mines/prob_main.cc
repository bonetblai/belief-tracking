/*
 *  Copyright (C) 2014 Universidad Simon Bolivar
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
#include <cstdlib>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <math.h>

#include <dai/alldai.h>

extern "C" {
#include "c_api.h"
};

using namespace std;


struct cell_t {
    bool mine_;
    int nmines_;
    cell_t() : mine_(false), nmines_(0) { }
};

struct minefield_t {
    int nrows_;
    int ncols_;
    int ncells_;
    int nmines_;
    int num_remaining_mines_;

    vector<cell_t> cells_;

    minefield_t(int nrows, int ncols, int nmines)
      : nrows_(nrows), ncols_(ncols), ncells_(nrows_ * ncols_), nmines_(nmines) {
    }

    int num_remaining_mines() const { return num_remaining_mines_; }

    void sample(int initial_cell) {
        // do not place a mine at the initial cell or surrounding cells
        // calculate places where a mine can be placed
        vector<int> available_cells(ncells_, 0);
        set<int> forbidden;
        do {
            forbidden.clear();
            available_cells = vector<int>(ncells_, 0);
            for( int i = 0; i < ncells_; ++i )
                available_cells[i] = i;

            int r = initial_cell / ncols_, c = initial_cell % ncols_;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = r + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = c + dc;
                    int cell = nr * ncols_ + nc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    forbidden.insert(-(1 + cell));
                }
            }
            for( set<int>::iterator it = forbidden.begin(); it != forbidden.end(); ++it ) {
                int pos = -(*it) - 1;
                available_cells[pos] = available_cells.back();
                available_cells.pop_back();
            }
        } while( (int)available_cells.size() < nmines_ );

        // available_cells contains the cells where mines can be placed.
        // Place mines in random cells. For each placed mines, increase
        // counter of surronding mines in surrounding cells.
        cells_ = vector<cell_t>(ncells_);
        for( int num_placed_mines = 0; num_placed_mines < nmines_; ++num_placed_mines ) {
            int pos = lrand48() % available_cells.size();
            int cell = available_cells[pos];
            available_cells[pos] = available_cells.back();
            available_cells.pop_back();
            assert(!cells_[cell].mine_);
            cells_[cell].mine_ = true;
            int r = cell / ncols_, c = cell % ncols_;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = r + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = c + dc;
                    int ncell = nr * ncols_ + nc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    if( cell == ncell ) continue;
                    ++cells_[ncell].nmines_;
                }
            }
        }
        num_remaining_mines_ = nmines_;
    }

    int flag_cell(int cell, bool verbose) {
        int r = cell / ncols_, c = cell % ncols_;
        if( verbose ) cout << "flag_cell(" << c << "," << r << "): mine=" << (cells_[cell].mine_ ? 1 : 0) << endl;
        num_remaining_mines_ -= cells_[cell].mine_ ? 1 : 0;
        return -1;
    }

    int open_cell(int cell, bool verbose) {
        int r = cell / ncols_, c = cell % ncols_;
        if( verbose ) cout << "open_cell(" << c << "," << r << "): mine=" << (cells_[cell].mine_ ? 1 : 0) << ", #mines=" << cells_[cell].nmines_ << endl;
        return cells_[cell].mine_ ? 9 : cells_[cell].nmines_;
    }

    int apply_action(int action, bool verbose) {
        int cell = agent_get_cell(action);
        if( agent_is_flag_action(action) ) {
            return flag_cell(cell, verbose);
        } else {
            return open_cell(cell, verbose);
        }
    }

    void print(ostream &os, bool formatted = false) const {
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                if( cells_[cell].mine_ )
                    os << " *";
                else
                    os << " " << cells_[cell].nmines_;
            }
            if( formatted ) os << endl;
        }
    }
};

// divergence measures between distributions
pair<float, bool> kl_divergence(const vector<float> &P, const vector<float> &Q) {
    assert(P.size() == Q.size());
    float kl = 0;
    for( size_t i = 0; i < P.size(); ++i ) {
        float p = P[i];
        float q = Q[i];
        if( (q == 0) && (p != 0) ) return make_pair(kl, false);
        if( p == 0 ) continue;
        kl += p * (log(p) - log(q));
    }
    return make_pair(kl, true);
}

pair<float, bool> js_divergence(const vector<float> &P, const vector<float> &Q) {
    assert(P.size() == Q.size());
    vector<float> M(P.size(), 0);
    for( size_t i = 0; i < M.size(); ++i )
        M[i] = (P[i] + Q[i]) / 2;
    pair<float, bool> kl1 = kl_divergence(P, M);
    pair<float, bool> kl2 = kl_divergence(Q, M);
    assert(kl1.second && kl2.second);
    return make_pair((kl1.first + kl2.first) / 2, true);
}

// probabilistic belief tracking for minesweeper
struct ms_pbt_t {
    // grid dimension, num of mines, and noisy flag
    int nrows_;
    int ncols_;
    int nmines_;
    bool noisy_;

    // variables, factors and centers for each beam
    vector<dai::Var> variables_;
    vector<dai::Factor> factors_;
    vector<int> centers_;

    // computation of marginals in factor model
    mutable int ninferences_;
    mutable vector<int> indices_for_updated_factors_;
    mutable vector<dai::Factor> marginals_;
    mutable dai::Factor full_joint_;
    mutable dai::InfAlg *inference_algorithm_;

    // inference algorithm and parameters
    string algorithm_;
    string options_;

    // data for inference algorithms
    dai::PropertySet libdai_options_;
    string edbp_tmp_path_;
    string edbp_factors_fn_;
    string edbp_evid_fn_;
    string edbp_output_fn_;
    int edbp_max_iter_;

    // variables for game play
    set<int> plays_;
    int nflags_;

    // action selection
    mutable int nguesses_;
    mutable float best_prob_for_open_;
    mutable float best_prob_for_flag_;
    mutable vector<int> best_for_open_;
    mutable vector<int> best_for_flag_;

    ms_pbt_t(int nrows, int ncols, int nmines, bool noisy, const string &inference_algorithm, const std::string &edbp_tmp_path = "")
      : nrows_(nrows), ncols_(ncols), nmines_(nmines), noisy_(noisy), edbp_tmp_path_(edbp_tmp_path) {
        // create binary variables for each cell in the grid
        variables_ = vector<dai::Var>(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc )
            variables_[loc] = dai::Var(loc, 2);

        // create one factor for each location. The variables
        // in the factor are the variables for the location
        // surrounding the factor, including the variable
        // for the "center" location. Also set up the center
        // for each factor.
        centers_ = vector<int>(nrows_ * ncols_);
        factors_ = vector<dai::Factor>(nrows_ * ncols_);
        marginals_ = vector<dai::Factor>(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            int row = loc / ncols_, col = loc % ncols_;
            vector<dai::Var> vars;
            for( int dr = -1; dr < 2; ++dr ) {
                int nr = row + dr;
                if( (nr < 0) || (nr >= nrows_) ) continue;
                for( int dc = -1; dc < 2; ++dc ) {
                    int nc = col + dc;
                    if( (nc < 0) || (nc >= ncols_) ) continue;
                    if( (dr == 0) && (dc == 0) ) centers_[loc] = vars.size();
                    vars.push_back(variables_[nr * ncols_ + nc]);
                }
            }
            dai::VarSet varset(vars.begin(), vars.end());
            factors_[loc] = dai::Factor(varset);
            marginals_[loc] = dai::Factor(variables_[loc]);
            //cout << "Factor[row=" << row << ",col=" << col << "]=" << p << endl;
        }
        cout << "pbt: factor graph:"
             << " #variables=" << variables_.size()
             << ", #factors=" << factors_.size()
             << endl;

        set_inference_algorithm(inference_algorithm);

        // create inference algorithm
        create_and_initialize_inference_algorithm();
    }
    ~ms_pbt_t() {
        destroy_inference_algorithm();
    }

    void set_inference_algorithm(const string &inference_algorithm) {
        cout << "setting inference algorithm to '" << inference_algorithm << "'" << endl;
        parse_inference_algorithm(inference_algorithm);

        // if edbp algorithm, create evidence file and set filenames
        if( algorithm_ == "edbp" ) {
            int pid = getpid();
            assert(edbp_tmp_path_.empty() || (edbp_tmp_path_.back() == '/'));
            edbp_factors_fn_ = edbp_tmp_path_ + "dummy" + to_string(pid) + ".factors";
            edbp_evid_fn_ = edbp_tmp_path_ + "dummy" + to_string(pid) + ".evid";
            edbp_output_fn_ = "/dev/null";
            system((string("echo 0 > ") + edbp_evid_fn_).c_str());
            if( libdai_options_.hasKey("maxiter") )
                edbp_max_iter_ = libdai_options_.getStringAs<size_t>("maxiter");
            else
                edbp_max_iter_ = 10;
        }
    }

    void clean_inference_algorithm() {
        if( algorithm_ == "edbp" ) {
            unlink(edbp_factors_fn_.c_str());
            unlink((edbp_factors_fn_ + ".MAR").c_str());
            unlink(edbp_evid_fn_.c_str());
            if( edbp_output_fn_ != "/dev/null" )
                unlink(edbp_output_fn_.c_str());
        }
    }

    void parse_inference_algorithm(const string &inference_algorithm) {
        size_t first_par = inference_algorithm.find_first_of('(');
        size_t last_par = inference_algorithm.find_first_of(')');
        assert(first_par != string::npos);
        assert(last_par != string::npos);
        assert(last_par == inference_algorithm.size() - 1);

        algorithm_ = string(inference_algorithm, 0, first_par);
        string options(inference_algorithm, first_par + 1, last_par - first_par - 1);

        // parameter types
        map<string, string> parameter_type;
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
        vector<string> tokens = dai::tokenizeString(options, false, ", ");
        for( int i = 0; i < int(tokens.size()); ++i ) {
            size_t equal_pos = tokens[i].find_first_of('=');
            assert(equal_pos != string::npos);
            string parameter(tokens[i], 0, equal_pos);
            string value(tokens[i], equal_pos + 1);

            // string paramenters
            if( parameter_type.find(parameter) == parameter_type.end() ) {
                cout << "warning: parameter '" << parameter << "' not recognized" << endl;
            } else {
                const string &type = parameter_type[parameter];
                if( type == "string" )
                    libdai_options_ = libdai_options_(parameter, value);
                else if( type == "double" )
                    libdai_options_ = libdai_options_(parameter, atof(value.c_str()));
                else if( type == "boolean" )
                    libdai_options_ = libdai_options_(parameter, value == "true");
                else if( type == "size_t" )
                    libdai_options_ = libdai_options_(parameter, size_t(atol(value.c_str())));
                else
                    cout << "warning: type '" << type << "' not supported" << endl;
            }
        }
    }

    void create_and_initialize_inference_algorithm() {
        if( algorithm_ != "edbp" ) {
            dai::FactorGraph factor_graph(factors_);
            if( algorithm_ == "jt" ) {
                inference_algorithm_ = new dai::JTree(factor_graph, libdai_options_);
            } else if( algorithm_ == "bp" ) {
                inference_algorithm_ = new dai::BP(factor_graph, libdai_options_);
            } else if( algorithm_ == "fbp" ) {
                inference_algorithm_ = new dai::FBP(factor_graph, libdai_options_);
            } else if( algorithm_ == "cbp" ) {
                inference_algorithm_ = new dai::CBP(factor_graph, libdai_options_);
            } else if( algorithm_ == "lc" ) {
                inference_algorithm_ = new dai::LC(factor_graph, libdai_options_);
            } else if( algorithm_ == "mr" ) {
                inference_algorithm_ = new dai::MR(factor_graph, libdai_options_);
            } else if( algorithm_ == "hak" ) {
                inference_algorithm_ = new dai::HAK(factor_graph, libdai_options_);
            } else if( algorithm_ == "tree-ep" ) {
                inference_algorithm_ = new dai::TreeEP(factor_graph, libdai_options_);
            } else {
                std::cout << "error: unrecognized inference algorithm '" << algorithm_ << "'" << std::endl;
                exit(-1);
            }
            std::cout << "Properties: " << inference_algorithm_->printProperties() << std::endl;
            inference_algorithm_->init();
        }
    }

    void destroy_inference_algorithm() {
        delete inference_algorithm_;
        inference_algorithm_ = 0;
    }

    void print_factor(ostream &os, int loc, const dai::Factor &factor, const string &name) const {
        os << "variables[loc=" << loc << "]=";
        for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it )
            os << " " << *it;
        os << endl << name << "[loc=" << loc << "]=" << endl;
        for( int j = 0; j < int(factor.nrStates()); ++j ) {
            os << "   ";
            map<dai::Var, size_t> states = dai::calcState(factor.vars(), j);
            for( dai::VarSet::const_iterator it = factor.vars().begin(); it != factor.vars().end(); ++it )
                os << " " << setw(2) << *it << "=" << states[*it];
            os << ":  j=" << setw(3) << j << ",  nbits=" << "na" << ",  value=" << factor[j] << endl;
        }
    }

    void print_factor_edbp(ostream &os, const dai::Factor &factor) const {
        os << factor.nrStates() << endl;
        for( int j = 0; j < int(factor.nrStates()); ++j )
            os << " " << factor[j];
        os << endl;
    }

    // reset all factors and game-play variables
    void reset() {
        indices_for_updated_factors_.clear();
        indices_for_updated_factors_.reserve(nrows_ * ncols_);
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            dai::Factor &factor = factors_[loc];
            float p = 1.0 / (1 << factor.vars().size());
            for( int j = 0; j < (1 << factor.vars().size()); ++j )
                factor.set(j, p);
            indices_for_updated_factors_.push_back(loc);
        }
        plays_.clear();
        nflags_ = 0;
        nguesses_ = 0;
        ninferences_ = 0;
        best_for_open_.clear();
        best_for_flag_.clear();
    }

    // update factors for obtained obs for cell
    void update_factors(bool flag_action, int cell, int obs) {
        assert(plays_.find(cell) == plays_.end());
        plays_.insert(cell);
        //int row = cell / ncols_, col = cell % ncols_;
        //cout << "pbt: flag=" << flag_action
        //     << ", cell=" << cell << ":(" << col << "," << row << ")"
        //     << ", obs=" << obs << endl;
        if( !flag_action ) {
            int center = centers_[cell];
            //cout << "center=" << center << ", var=" << factors_[cell].vars().var(center) << endl;
            dai::Factor &factor = factors_[cell];
            for( int j = 0; j < int(factor.nrStates()); ++j ) {
                int popcount = __builtin_popcount(j);
                if( !noisy_ ) {
                    if( popcount != obs ) factor.set(j, 0);
                } else {
                    // noisy update
                    if( popcount == obs ) {
                        factor.set(j, factor[j] * .95);
                    } else if( (popcount == obs - 1) || (popcount == obs + 1) ) {
                        factor.set(j, factor[j] * .05);
                    } else {
                        factor.set(j, 0);
                    }
                }
                if( (j & (1 << center)) != 0 ) factor.set(j, 0);
            }
            indices_for_updated_factors_.push_back(cell);

            // if obs is 0 and not noisy, neighboring cells don't have mines
            if( !noisy_ && (obs == 0) ) {
                if( best_prob_for_open_ < 1 ) {
                    best_prob_for_open_ = 1;
                    best_for_open_.clear();
                }
                int row = cell / ncols_, col = cell % ncols_;
                for( int dr = -1; dr < 2; ++dr ) {
                    int nr = row + dr;
                    if( (nr < 0) || (nr >= nrows_) ) continue;
                    for( int dc = -1; dc < 2; ++dc ) {
                        int nc = col + dc;
                        if( (dr == 0) && (dc == 0) ) continue;
                        if( (nc < 0) || (nc >= ncols_) ) continue;
                        int nloc = nr * ncols_ + nc;
                        //cout << "adding (" << nc << "," << nr << ")" << endl;
                        best_for_open_.push_back(nloc);
                    }
                }
            }
        } else {
            ++nflags_;
        }
    }

    void calculate_marginals(bool print_marginals = false) const {
        if( !indices_for_updated_factors_.empty() ) {
            ++ninferences_;
            if( algorithm_ == "edbp" ) {
                apply_inference_edbp();
                extract_marginals_from_inference_edbp(print_marginals);
            } else {
                apply_inference_libdai();
                extract_marginals_from_inference_libdai(print_marginals);
            }
            indices_for_updated_factors_.clear();
        }
    }

    void extract_marginals_from_inference_libdai(bool print_marginals = false) const {
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            marginals_[loc] = inference_algorithm_->belief(variables_[loc]);
            if( print_marginals )
                print_factor(cout, loc, marginals_[loc], "libdai::marginals_");
        }
    }

    void apply_inference_libdai() const {
        assert(inference_algorithm_ != 0);
        assert(!indices_for_updated_factors_.empty());
        dai::VarSet variables;
        for( int i = 0; i < int(indices_for_updated_factors_.size()); ++i ) {
            int index = indices_for_updated_factors_[i];
            inference_algorithm_->fg().setFactor(index, factors_[index]);
            variables |= factors_[index].vars();
        }
        inference_algorithm_->init(variables);
        inference_algorithm_->run();
    }

    void extract_marginals_from_inference_edbp(bool print_marginals = false) const {
        ifstream ifs(edbp_factors_fn_ + ".MAR");

        string buff;
        ifs >> buff;
        assert(buff == "MAR");

        bool more_results = true;
        while( more_results ) {
            int nlines = 0;
            ifs >> nlines;
            assert(nlines == 1);

            int nfactors = 0;
            ifs >> nfactors;
            assert(nfactors == nrows_ * ncols_);
            for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
                int nstates = 0;
                ifs >> nstates;
                assert(nstates == 2); // binary variables
                for( int j = 0; j < nstates; ++j ) {
                    float p = 0;
                    ifs >> p;
                    marginals_[loc].set(j, p);
                }
                if( print_marginals ) print_factor(cout, loc, marginals_[loc], "edbp::marginals_");
            }

            // check if there are more results
            string marker;
            ifs >> marker;
            more_results = marker == "-BEGIN-";
        }
        ifs.close();
    }

    void apply_inference_edbp() const {
        // create model in given file
        ofstream ofs(edbp_factors_fn_);
        ofs << "MARKOV"
            << endl
            << nrows_ * ncols_
            << endl;
        for( int loc = 0; loc < nrows_ * ncols_; ++loc )
            ofs << 2 << (loc < nrows_ * ncols_ - 1 ? " " : "");
        ofs << endl << nrows_ * ncols_ << endl;
        for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
            const dai::Factor &factor = factors_[loc];
            ofs << factor.vars().size();
            for( dai::VarSet::const_reverse_iterator it = factor.vars().rbegin(); it != factor.vars().rend(); ++it ) {
                ofs << " " << it->label();
            }
            ofs << endl;
        }
        ofs << endl;
        for( int loc = 0; loc < int(factors_.size()); ++loc ) {
            print_factor_edbp(ofs, factors_[loc]);
            ofs << endl;
        }
        ofs.close();

        // call edbp solver
        string edbp_cmd = string("~/software/edbp/solver") + " " + edbp_factors_fn_ + " " + edbp_evid_fn_ + " " + to_string(edbp_max_iter_) + " 0 MAR 2>" + edbp_output_fn_;
        system(edbp_cmd.c_str());
    }

    // compute full joint by multiplying factors (exponential)
    void compute_full_joint() const {
        full_joint_ = dai::Factor();
        for( int i = 0; i < nrows_ * ncols_; ++i )
            full_joint_ *= factors_[i];
        full_joint_.normalize();
    }

    // return the marginal distribution of the full joint over a cell variable
    pair<float, float> full_joint_marginal(int cell) const {
        dai::Factor marginal = full_joint_.marginal(variables_[cell]);
        return make_pair(float(marginal[0]), float(marginal[1]));
    }

    // recommend action using information in the marginals
    int agent_get_action() const {
        int action = -1;
        if( (best_prob_for_open_ == 1) && !best_for_open_.empty() ) {
            int cell = -1;
            while( (cell == -1) && !best_for_open_.empty() ) {
                int index = lrand48() % best_for_open_.size();
                int candidate = best_for_open_[index];
                best_for_open_[index] = best_for_open_.back();
                best_for_open_.pop_back();
                if( plays_.find(candidate) == plays_.end() )
                    cell = candidate;
            }
            if( cell != -1 ) action = cell + (nrows_ * ncols_);
        }

        if( (action == -1) && (best_prob_for_flag_ == 1) && !best_for_flag_.empty() ) {
            int cell = -1;
            while( (cell == -1) && !best_for_flag_.empty() ) {
                int index = lrand48() % best_for_flag_.size();
                int candidate = best_for_flag_[index];
                best_for_flag_[index] = best_for_flag_.back();
                best_for_flag_.pop_back();
                if( plays_.find(candidate) == plays_.end() )
                    cell = candidate;
            }
            if( cell != -1 ) action = cell;
        }

        if( action == -1 ) {
            calculate_marginals(false);
            best_for_open_.clear();
            best_for_flag_.clear();
            best_prob_for_open_ = 0;
            best_prob_for_flag_ = 0;
            for( int loc = 0; loc < nrows_ * ncols_; ++loc ) {
                if( plays_.find(loc) != plays_.end() ) continue; // cell had been already played
                const dai::Factor &marginal = marginals_[loc];
                assert(marginal.nrStates() == 2);
                if( best_for_open_.empty() || (marginal[0] >= best_prob_for_open_) ) {
                    if( best_for_open_.empty() || (marginal[0] > best_prob_for_open_) ) {
                        best_prob_for_open_ = marginal[0];
                        best_for_open_.clear();
                    }
                    best_for_open_.push_back(loc);
                }
                if( (nflags_ < nmines_) && (best_for_flag_.empty() || (marginal[1] >= best_prob_for_flag_)) ) {
                    if( best_for_flag_.empty() || (marginal[1] > best_prob_for_flag_) ) {
                        best_prob_for_flag_ = marginal[1];
                        best_for_flag_.clear();
                    }
                    best_for_flag_.push_back(loc);
                }
            }

            cout << "best for open: p=" << best_prob_for_open_ << ", sz=" << best_for_open_.size() << ", loc=";
            for( int i = 0; i < int(best_for_open_.size()); ++i )
                cout << best_for_open_[i] << ",";
            cout << endl << "best for flag: p=" << best_prob_for_flag_ << ", sz=" << best_for_flag_.size() << ", loc=";
            for( int i = 0; i < int(best_for_flag_.size()); ++i )
                cout << best_for_flag_[i] << ",";
            cout << endl;

            // prioritize open over flag actions
            if( !best_for_open_.empty() && (best_prob_for_open_ >= best_prob_for_flag_) ) {
                nguesses_ += best_prob_for_open_ < 1;
                int index = lrand48() % best_for_open_.size();
                int cell = best_for_open_[index];
                best_for_open_[index] = best_for_open_.back();
                best_for_open_.pop_back();
                action = cell + (nrows_ * ncols_);
            } else if( !best_for_flag_.empty() && (best_prob_for_open_ < best_prob_for_flag_) ) {
                nguesses_ += best_prob_for_flag_ < 1;
                int index = lrand48() % best_for_flag_.size();
                int cell = best_for_flag_[index];
                best_for_flag_[index] = best_for_flag_.back();
                best_for_flag_.pop_back();
                action = cell;
            }
        }

        assert(action != -1);
        return action;
    }

    bool is_flag_action(int action) {
        return action < nrows_ * ncols_ ? true : false;
    }
    int agent_get_cell(int action) {
        return is_flag_action(action) ? action : action - (nrows_ * ncols_);
    }
};

void usage(ostream &os) {
    os << endl
       << "Usage: mines [{-t | --ntrials} <ntrials>]" << endl
       << "             [{-r | --nrows} <nrows>]" << endl
       << "             [{-c | --ncols} <ncols>]" << endl
       << "             [{-m | --nmines} <nmines>]" << endl
       << "             [{-s | --seed} <seed>]" << endl
       << "             [{-v | --verbose}]" << endl
       << "             [{-? | --help}]" << endl
       << endl
       << "where <ntrials> is a non-negative integer telling the number of games to" << endl
       << "play (default is 1), <nrows> and <ncols> are positive integers telling" << endl
       << "the dimensions of the minefield (default is 16x16), <nmines> is a positive" << endl
       << "integer telling the number of hidden mines in the minefield (default is 40)," << endl
       << "and <seed> is an integer for setting the seed of the random number generator" << endl
       << "(default is 0)." << endl
       << endl
       << "For example," << endl
       << endl
       << "  ./mines -r 16 -c 16 -m 40 -t 100" << endl
       << endl
       << "performs an experiment consisting of 100 trials on a 16x16 minefield" << endl
       << "with 40 mines." << endl
       << endl;
}

int main(int argc, const char **argv) {
    int ntrials = 1;
    int nrows = 16;
    int ncols = 16;
    int nmines = 40;
    int seed = 0;
    bool verbose = false;
    string tmp_path = "";

    // inference algorithm
    //string inference_algorithm = "edbp(maxiter=10)";
    //string inference_algorithm = "jt(updates=HUGIN)";
    string inference_algorithm = "bp(updates=SEQRND,logdomain=false,tol=1e-3,maxiter=20,maxtime=1,damping=.2)";
    //string inference_algorithm = "cbp(updates=SEQRND,clamp=CLAMP_VAR,choose=CHOOSE_RANDOM,min_max_adj=10,bbp_props=,bbp_cfn=,recursion=REC_FIXED,tol=1e-3,rec_tol=1e-3,maxiter=100)";
    //string inference_algorithm = "lc(updates=SEQRND,cavity=FULL,logdomain=false,tol=1e-3,maxiter=100,maxtime=1,damping=.2)";
    //string inference_algorithm = "mr(updates=LINEAR,inits=RESPPROP,logdomain=false,tol=1e-3,maxiter=100,maxtime=1,damping=.2)";
    //string inference_algorithm = "hak(doubleloop=true,clusters=MIN,init=UNIFORM,tol=1e-3,maxiter=100,maxtime=1)";

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
        } else if( !strcmp(argv[0], "-t") || !strcmp(argv[0], "--ntrials") ) {
            ntrials = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-r") || !strcmp(argv[0], "--nrows") ) {
            nrows = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-c") || !strcmp(argv[0], "--ncols") ) {
            ncols = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-m") || !strcmp(argv[0], "--nmines") ) {
            nmines = atoi(argv[1]);
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
            usage(cout);
            exit(-1);
        } else {
            cout << "error: unexpected argument: " << argv[0] << endl;
            --argc;
            ++argv;
        }
    }

    // set seed
    cout << "SEED=" << seed << endl;
    unsigned short seeds[3];
    seeds[0] = seeds[1] = seeds[2] = seed;
    seed48(seeds);

    // create distributions
    if( !tmp_path.empty() && (tmp_path.back() != '/') ) tmp_path += '/';
    ms_pbt_t pbt(nrows, ncols, nmines, false, inference_algorithm, tmp_path);

    // run for the specified number of trials
    for( int trial = 0; trial < ntrials; ) {
        minefield_t minefield(nrows, ncols, nmines);
        agent_initialize(nrows, ncols, nmines);

        // inference and print marginals
        pbt.reset();

        bool win = true;
        vector<pair<int,int> > execution(nrows * ncols);
        for( int play = 0; play < nrows * ncols; ++play ) {
            int action = pbt.agent_get_action();
            bool is_flag_action = pbt.is_flag_action(action);
            int cell = pbt.agent_get_cell(action);
            cout << "Play: n=" << play
                 << ", type=" << (is_flag_action ? "FLAG" : "OPEN")
                 << ", cell=" << cell << ":(" << cell % ncols << "," << cell / ncols << ")"
                 << flush;

            // if this is first play, then it must be an open action. The minefield
            // gets sampled using the action's cell.
            if( play == 0 ) {
                assert(!pbt.is_flag_action(action));
                int cell = agent_get_cell(action);
                minefield.sample(cell);
                assert(!minefield.cells_[cell].mine_);
                cout << ", obs=0" << endl << endl;
                minefield.print(cout, true);
                cout << endl;
            }

            // obtain observation for this action. If first play, observation
            // must be zero.
            int obs = minefield.apply_action(action, verbose);
            assert((obs == 0) || (play > 0));
            if( play > 0 ) cout << ", obs=" << obs << endl;
            if( obs == 9 ) {
                cout << "**** BOOM!!! ****" << endl;
                win = false;
                break;
            }

            // update execution and agent's belief 
            execution[play] = make_pair(action, obs);

            // update agent's belief
            agent_update_state(agent_is_flag_action(action), agent_get_cell(action), obs);
            pbt.update_factors(pbt.is_flag_action(action), agent_get_cell(action), obs);
        }

        agent_increase_nguesses(pbt.nguesses_);
        agent_increase_ninferences(pbt.ninferences_);
        if( win )
            agent_declare_win(true);
        else
            agent_declare_lose(true);

        ++trial;
    }
    agent_finalize();
    pbt.clean_inference_algorithm();
    return 0;
}

