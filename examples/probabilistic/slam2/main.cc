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
#include "motion_model_sir.h"
#include "optimal_sir.h"
#include "motion_model_rbpf.h"
#include "optimal_rbpf.h"
//#include "ppcbt2.h"
#include "slam_particle_types.h"

using namespace std;


// static members
int coord_t::ncols_ = 0;
const cellmap_t *base_particle_t::base_ = 0;


void usage(ostream &os) {
    os << endl
       << "Usage: colorslam [{-t | --ntrials} <ntrials>]" << endl
       << "                 [{-r | --nrows} <nrows>]" << endl
       << "                 [{-c | --ncols} <ncols>]" << endl
       << "                 [{-l | --nlabels} <nlabels>]" << endl
       << "                 [{-s | --seed} <seed>]" << endl
       << "                 [{-v | --verbose}]" << endl
       << "                 [{-p | --policy} <policy>]" << endl
       << "                 [{-? | --help}]" << endl
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

    int gtype = -1;
    int ptype = 0;
    int execution_length = 10;

    --argc;
    ++argv;
    while( (argc > 0) && (**argv == '-') ) {
        if( !strcmp(argv[0], "-g") || !strcmp(argv[0], "--gtype") ) {
            gtype = atoi(argv[1]);
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
        } else if( !strcmp(argv[0], "-r") || !strcmp(argv[0], "--nrows") ) {
            nrows = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-n") || !strcmp(argv[0], "--nsteps") ) {
            nsteps = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-t") || !strcmp(argv[0], "--ntrials") ) {
            ntrials = atoi(argv[1]);
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
        } else if( !strcmp(argv[0], "-p") || !strcmp(argv[0], "--policy") ) {
            ptype = atoi(argv[1]);
            execution_length = atoi(argv[2]);
            argc -= 3;
            argv += 3;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
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

    // create cellmap
    if( (gtype >= 0) && (gtype < 2) ) {
        nrows = 1;
        ncols = 8;
        nlabels = 2;
        pa = 0.8;
        po = 0.9;
    } else if( (gtype >= 0) && (gtype < 3) ) {
        nrows = 10;
        ncols = 10;
        nlabels = 4;
        pa = 0.8; // CHECK
        po = 0.9; // CHECK
    }
    cellmap_t cellmap(nrows, ncols, nlabels, pa, po, 0.0, gtype == 2);

    // set static members
    coord_t::ncols_ = ncols;
    base_particle_t::base_ = &cellmap;

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
        } else if( name == "ppcbt_bp" ) {
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 50);
            pcbt->set_algorithm_and_options("HAK", opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));
            tracking_algorithms.push_back(pcbt);
        } else if( name == "ppcbt_hak" ) {
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 50);
            pcbt->set_algorithm_and_options("BP", opts("updates", string("SEQRND"))("logdomain", false)("tol", 1e-9)("maxiter", (size_t)10000));
            tracking_algorithms.push_back(pcbt);
#endif
        } else if( name == "sis" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new SIS_t<sis_slam_particle_t, cellmap_t>(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "mm_sir2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new motion_model_SIR_t<motion_model_sir_slam_particle_t, cellmap_t>(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "opt_sir2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t> *cdf = new cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t>(cellmap);
            tracking_algorithms.push_back(new optimal_SIR_t<optimal_sir_slam_particle_t, cellmap_t, cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t> >(name + "_" + to_string(nparticles), cellmap, *cdf, nparticles));
        } else if( name == "mm_rbpf2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new motion_model_RBPF_t<motion_model_rbpf_slam_particle_t, cellmap_t>(name + "_" + to_string(nparticles), cellmap, nparticles));
        } else if( name == "opt_rbpf2" ) {
            nparticles = token != 0 ? atoi(token) : nparticles;
            tracking_algorithms.push_back(new optimal_RBPF_t<optimal_rbpf_slam_particle_t, cellmap_t>(name + "_" + to_string(nparticles), cellmap, nparticles));
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

    // set initial loc
    cellmap.initial_loc_ = 0;

    // set labels
    vector<int> labels;
    if( (gtype >= 0) && (gtype < 2) ) {
        if( gtype == 0 ) { // 1x8 grid with fixed labels
            int _labels[] = { 0, 1, 0, 1, 0, 1, 0, 1 };
            labels = vector<int>(&_labels[0], &_labels[8]);
        } else { // 1x8 grid with random labels
            cellmap.sample_labels(labels);
        }
    } else if( (gtype >= 0) && (gtype < 3) ) {
        if( gtype == 2 ) { // 10x10 grid with fixed labels: 0=empty, 1=wall, 2=opened door, 3=closed door
            int _labels[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                              1, 0, 0, 2, 0, 3, 0, 0, 0, 1,
                              1, 0, 0, 1, 0, 1, 0, 0, 0, 1,
                              1, 3, 1, 1, 0, 1, 1, 1, 2, 1,
                              1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                              1, 1, 1, 1, 0, 1, 1, 1, 3, 1,
                              1, 0, 0, 2, 0, 1, 0, 0, 0, 1,
                              1, 0, 0, 1, 0, 1, 0, 0, 0, 1,
                              1, 0, 0, 1, 0, 2, 0, 0, 0, 1,
                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
            labels = vector<int>(&_labels[0], &_labels[100]);
            cellmap.initial_loc_ = coord_t(1, 1).as_index();
        }
    } else { // general grid with random labels
        cellmap.sample_labels(labels);
    }

    // set execution
    cellmap_t::execution_t fixed_execution;
    if( ptype == 0 ) {
        if( gtype < 2 ) {
            fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, -1));
            fixed_execution.push_back(cellmap_t::execution_step_t(1, 1, cellmap_t::right));
            fixed_execution.push_back(cellmap_t::execution_step_t(2, 0, cellmap_t::right));
            fixed_execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::right));
            fixed_execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::right));
            fixed_execution.push_back(cellmap_t::execution_step_t(4, 0, cellmap_t::right));
            fixed_execution.push_back(cellmap_t::execution_step_t(5, 1, cellmap_t::right));
            fixed_execution.push_back(cellmap_t::execution_step_t(6, 0, cellmap_t::right));
            fixed_execution.push_back(cellmap_t::execution_step_t(7, 1, cellmap_t::right));
            fixed_execution.push_back(cellmap_t::execution_step_t(6, 0, cellmap_t::left));
            fixed_execution.push_back(cellmap_t::execution_step_t(5, 1, cellmap_t::left));
            fixed_execution.push_back(cellmap_t::execution_step_t(4, 0, cellmap_t::left));
            fixed_execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::left));
            fixed_execution.push_back(cellmap_t::execution_step_t(2, 0, cellmap_t::left));
            fixed_execution.push_back(cellmap_t::execution_step_t(1, 1, cellmap_t::left));
            fixed_execution.push_back(cellmap_t::execution_step_t(0, 0, cellmap_t::left));
        }
    } else {
        cellmap.compute_random_execution(labels, 0, execution_length, fixed_execution);
    }

    // run for the specified number of trials
    for( int trial = 0; trial < ntrials; ++trial ) {
        cellmap_t::execution_t output_execution;
        if( !fixed_execution.empty() )
            cellmap.run_execution(labels, fixed_execution, output_execution, tracking_algorithms);
        else
            cellmap.run_execution(labels, output_execution, cellmap.initial_loc_, nsteps, policy, tracking_algorithms);

        // calculate final marginals
        cout << "#output size = " << output_execution.size() << endl;
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            tracking_algorithms[i]->calculate_marginals();

        // compute and print scores
        vector<cellmap_t::score_t> scores;
        cellmap.compute_scores(output_execution, tracking_algorithms, scores);
        for( size_t i = 0; i < scores.size(); ++i ) {
            cout << "# score(" << setw(size_longest_name) << tracking_algorithms[i]->name_ << "): score=" << scores[i] << endl;
        }

        // print final (tracked) map and location
        vector<int> map_values;
        cout << "# final(" << setw(size_longest_name) << "real" << "): map=";
        cellmap.print_labels(cout);
        cout << ", loc=" << coord_t(output_execution.back().loc_) << endl;
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            cout << "# final(" << setw(size_longest_name) << tracking_algorithms[i]->name_ << "): map=[";
            for( int var = 0; var < nrows * ncols; ++var ) {
                tracking_algorithms[i]->MAP_on_var(var, map_values);
                if( map_values.size() == 1 ) {
                    cout << " " << map_values.back();
                } else {
#if 1
                    cout << " *";
#else
                    cout << " {";
                    for( size_t k = 0; k < map_values.size(); ++k ) {
                        cout << map_values[k];
                        if( k < map_values.size() - 1 ) cout << ",";
                    }
                    cout << "}";
#endif
                }
                if( (var + 1 < nrows * ncols) && (((var + 1) % ncols) == 0) )
                    cout << " |";
            }

            cout << "], loc=";
            tracking_algorithms[i]->MAP_on_var(nrows * ncols, map_values);
            if( map_values.size() == 1 ) {
                cout << coord_t(map_values.back()) << endl;
            } else {
                cout << " {";
                for( size_t k = 0; k < map_values.size(); ++k ) {
                    cout << coord_t(map_values[k]);
                    if( k < map_values.size() - 1 ) cout << ",";
                }
                cout << "}" << endl;
            }
        }

        // generate R plots
        cout << "library(\"reshape2\");" << endl << "library(\"ggplot2\");" << endl;
        for( size_t i = 0; i < tracking_algorithms.size(); ++i )
            cellmap.generate_R_plot(cout, *tracking_algorithms[i]);
    }

    return 0;
}

