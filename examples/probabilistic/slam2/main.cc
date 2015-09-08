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
#include <iomanip>
#include <sstream>
#include <string.h>
#include <set>
#include <vector>
#include <stdlib.h>
#include <math.h>

#include "tracking.h"
#include "cellmap.h"
#include "slam_action_selection.h"

#include "pcbt.h"

#include "sis.h"
#include "motion_model_sir2.h"
#include "optimal_sir2.h"
#include "rbpf2.h"
//#include "ppcbt2.h"
#include "slam_particle_types.h"

using namespace std;


// static members
int coord_t::ncols_ = 0;
const cellmap_t *slam_particle_t::base_ = 0;


void usage(ostream &os) {
    os << endl
       << "Usage: maplearning [{-t | --ntrials} <ntrials>]" << endl
       << "                   [{-r | --nrows} <nrows>]" << endl
       << "                   [{-c | --ncols} <ncols>]" << endl
       << "                   [{-l | --nlabels} <nlabels>]" << endl
       << "                   [{-s | --seed} <seed>]" << endl
       << "                   [{-v | --verbose}]" << endl
       << "                   [{-p | --policy} <policy>]" << endl
       << "                   [{-? | --help}]" << endl
       << endl
       << "where <ntrials> is a non-negative integer telling the number of games to" << endl
       << "play (default is 1), <nrows> and <ncols> are positive integers telling" << endl
       << "the dimensions of the minefield (default is 16x16), <nmines> is a positive" << endl
       << "integer telling the number of hidden mines in the minefield (default is 40)," << endl
       << "<seed> is an integer for setting the seed of the random number generator" << endl
       << "(default is 0), <policy> is a string describing the policy to use (default" << endl
       << "is \"base-policy:direct\"), and <width> and <depth> are parameters for the" << endl
       << "policy (the default policy is parameter-free)." << endl
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
    int nrows = 1;
    int ncols = 8;
    int nlabels = 2;
    int nsteps = 0;
    int seed = 0;
    bool verbose = false;
    vector<string> tracking_algorithms_str;

    float pa = 0.8;
    float po = 0.9;

    --argc;
    ++argv;
    while( (argc > 0) && (**argv == '-') ) {
        if( !strcmp(argv[0], "-t") || !strcmp(argv[0], "--ntrials") ) {
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
        } else if( !strcmp(argv[0], "-l") || !strcmp(argv[0], "--nlabels") ) {
            nlabels = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-n") || !strcmp(argv[0], "--nsteps") ) {
            nsteps = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "--pa") ) {
            pa = strtod(argv[1], 0);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "--po") ) {
            po = strtod(argv[1], 0);
            argc -= 2;
            argv += 2;
        } else if( !strncmp(argv[0], "--tracking=", 11) ) {
            char *str = strdup(&argv[0][11]);
            char *token = strtok(str, ",");
            while( token != 0 ) {
                tracking_algorithms_str.push_back(token);
                token = strtok(0, ",");
            }
            free(str);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-v") || !strcmp(argv[0], "--verbose") ) {
            verbose = true;
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-p") || !strcmp(argv[0], "--policy") ) {
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-?") || !strcmp(argv[0], "--help") ) {
            usage(cerr);
            exit(-1);
        } else {
            cerr << "error: unexpected argument: " << argv[0] << endl;
            --argc;
            ++argv;
        }
    }

    // set seed
    unsigned short seeds[3];
    seeds[0] = seeds[1] = seeds[2] = seed;
    seed48(seeds);

    // create cellmap and tracking algorithms
    cellmap_t cellmap(nrows, ncols, nlabels, pa, po, 0.0);

    // set static members
    coord_t::ncols_ = ncols;
    slam_particle_t::base_ = &cellmap;

    // tracking algorithms
    vector<tracking_t<cellmap_t>*> tracking_algorithms;
    dai::PropertySet opts;
    int nparticles = 100;
    int memory = 0;

    for( size_t i = 0; i < tracking_algorithms_str.size(); ++i ) {
        char *str = strdup(tracking_algorithms_str[i].c_str());
        string name = strtok(str, ":");
        char *token = strtok(0, ":");
        if( name == "jt" ) {
            memory = token != 0 ? atoi(token) : memory;
            pcbt_t<cellmap_t> *pcbt = new pcbt_t<cellmap_t>(name + "_" + to_string(memory), cellmap, memory);
            pcbt->set_algorithm_and_options("JT", opts("updates", string("HUGIN")));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "bp" ) {
            memory = token != 0 ? atoi(token) : memory;
            pcbt_t<cellmap_t> *pcbt = new pcbt_t<cellmap_t>(name + "_" + to_string(memory), cellmap, memory);
            pcbt->set_algorithm_and_options("BP", opts("updates", string("SEQRND"))("logdomain", false)("tol", 1e-9)("maxiter", (size_t)10000));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "hak" ) {
            memory = token != 0 ? atoi(token) : memory;
            pcbt_t<cellmap_t> *pcbt = new pcbt_t<cellmap_t>(name + "_" + to_string(memory), cellmap, memory);
            pcbt->set_algorithm_and_options("HAK", opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));
            tracking_algorithms.push_back(pcbt);
#if 0 // ppcbt2_t
        } else if( name == "ppcbt_jt" ) {
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 1000);
            pcbt->set_algorithm_and_options("JT", opts("updates", string("HUGIN")));
            tracking_algorithms.push_back(pcbt);
#if 0 // ppcbt2_t
        } else if( name == "ppcbt_bp" ) {
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 50);
            pcbt->set_algorithm_and_options("HAK", opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "ppcbt_hak" ) {
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 50);
            pcbt->set_algorithm_and_options("BP", opts("updates", string("SEQRND"))("logdomain", false)("tol", 1e-9)("maxiter", (size_t)10000));
            tracking_algorithms.push_back(pcbt);
#endif
#endif
        } else if( name == "sis" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new SIS_t<sis_slam_particle_t, cellmap_t>(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "mm_sir2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new motion_model_SIR2_t<motion_model_sir2_slam_particle_t, cellmap_t>(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "opt_sir2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            cdf_t<optimal_sir2_slam_particle_t, cellmap_t> *cdf = new cdf_t<optimal_sir2_slam_particle_t, cellmap_t>(cellmap);
            tracking_algorithms.push_back(new optimal_SIR2_t<optimal_sir2_slam_particle_t, cellmap_t, cdf_t<optimal_sir2_slam_particle_t, cellmap_t> >(name + "_" + to_string(nparticles), cellmap, *cdf, nparticles));
        } else if( name == "rbpf2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new RBPF2_t<rbpf_slam_particle_t, cellmap_t>(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else {
            cerr << "warning: unrecognized tracking algorithm '" << name << "'" << endl;
        }
        free(str);
    }

    // compute longest name
    size_t size_longest_name = 0;
    for( size_t i = 0; i < tracking_algorithms.size(); ++i )
        size_longest_name = size_longest_name > tracking_algorithms[i]->name_.size() ? size_longest_name : tracking_algorithms[i]->name_.size();

    // action selection
    naive_action_selection_t policy(cellmap, .30);

    int labels[] = { 0, 1, 0, 1, 0, 1, 0, 1 };

    cellmap_t::execution_t fixed_execution;
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, -1));
    fixed_execution.push_back(cellmap_t::execution_step_t(1, 1, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(2, 0, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(4, 0, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(5, 1, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(6, 0, cellmap_t::right));
    fixed_execution.push_back(cellmap_t::execution_step_t(5, 1, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(4, 0, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(2, 0, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(1, 1, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, cellmap_t::left));
    fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, cellmap_t::left));

    // run for the specified number of trials
    for( int trial = 0; trial < ntrials; ++trial ) {
        cellmap_t::execution_t output_execution;
        if( (nsteps == 0) && (nrows == 1) && (ncols == 8) )
            cellmap.run_execution(labels, 8, fixed_execution, output_execution, tracking_algorithms);
        else
            cellmap.run_execution(labels, 0, output_execution, nsteps, policy, tracking_algorithms);

        // calculate final marginals
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            tracking_algorithms[i]->calculate_marginals();

        // compute and print scores
        vector<cellmap_t::score_t> scores;
        cellmap.compute_scores(output_execution, tracking_algorithms, scores);
        for( size_t i = 0; i < scores.size(); ++i ) {
            cout << "# score(" << setw(size_longest_name) << tracking_algorithms[i]->name_ << "): score=" << scores[i] << endl;
        }

        // print final (tracked) map and location
        cout << "# final(" << setw(size_longest_name) << "real" << "): map=[";
        for( int var = 0; var < nrows * ncols; ++var )
            cout << " " << cellmap.cells_[var].label_;
        cout << "], loc=" << output_execution.back().loc_ << endl;
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            cout << "# final(" << setw(size_longest_name) << tracking_algorithms[i]->name_ << "): map=[";
            for( int var = 0; var < nrows * ncols; ++var )
                cout << " " << tracking_algorithms[i]->MAP_on_var(var);
            cout << "], loc=" << tracking_algorithms[i]->MAP_on_var(nrows * ncols) << endl;
        }

        // generate R plots
        cout << "library(\"reshape2\");" << endl << "library(\"ggplot2\");" << endl;
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            cellmap.generate_R_plot(cout, *tracking_algorithms[i]);
    }

    return 0;
}




#if 0 // deprecated after main
            // compute and print exact marginals via junction tree
            marginal_t P1, Q1;
            //cout << "Marginals (exact: junction tree):" << endl;
            apply_junction_tree(P1, Q1);
            store_marginal(nrows_ * ncols_, &cellmap_t::jt_marginal, marginals);
            //print_marginals(cout, &cellmap_t::jt_marginal, 10);

            // approximate and print marginals
            marginal_t P2, Q2;
            //cout << "Marginals (approx. inference):" << endl;
            apply_approx_inference(P2, Q2);
            //print_marginals(cout, &cellmap_t::approx_inference_marginal);

            // calculate KL-divergence
            pair<float, bool> js1 = JS_divergence(P1, P2);
            pair<float, bool> js2 = JS_divergence(Q1, Q2);
            cout << "JS-divergence: P[jt,approx]=" << js1.first << ", Q[jt,approx]=" << js2.first << endl;

    void apply_approx_inference(marginal_t &P, marginal_t &Q) const {
        apply_approx_inference();
        P.clear();
        float mass = 0;
        for( int i = 0; i < nloc_; ++i ) {
            P.push_back(approx_inference_algorithm_->belief(approx_inference_fg_.var(i))[0]);
            mass = P.back();
        }
        for( int i = 0; i < nloc_; ++i ) P[i] /= mass;
        Q.clear();
        for( int i = 0; i < nloc_; ++i )
            Q.push_back(approx_inference_algorithm_->belief(approx_inference_fg_.var(nloc_))[i]);
    }

    // inference algorithms for MAP
    void apply_junction_tree_for_MAP(vector<size_t> &state) const {
        dai::PropertySet opts;
        jt_fg_ = dai::FactorGraph(factors_);
        jt_ = dai::JTree(jt_fg_, opts("updates", string("HUGIN"))("inference", string("MAXPROD")));
        jt_.init();
        jt_.run();
        state = jt_.findMaximum();
    }

        // Calculate MAP over cellmap labels
        vector<size_t> map_value;
        cellmap.apply_junction_tree_for_MAP(map_value);
        for( size_t i = 0; i < map_value.size(); ++i ) {
            cout << "v" << i << " = " << map_value[i];
            if( int(i) < nrows * ncols ) cout << " [label=" << cellmap.cells_[i].label_ << "]";
            cout << endl;
        }
#endif

