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

////#include "pcbt.h"

#include "sis.h"
#include "motion_model_sir.h"
#include "optimal_sir.h"
#include "rbpf.h"
////#include "ppcbt2.h"

#include "slam_particles.h"
#include "slam2_particles.h"

using namespace std;


// static members
int coord_t::ncols_ = 0;
const cellmap_t *base_particle_t::base_ = 0;
vector<vector<int> > rbpf_slam2_particle_t::slabels_;
vector<vector<int> > rbpf_slam2_particle_t::edbp_factor_indices_;
string inference_t::algorithm_;
string inference_t::options_;
dai::PropertySet inference_t::libdai_options_;
string inference_t::type_;
string inference_t::edbp_factors_fn_;
string inference_t::edbp_evid_fn_;
string inference_t::edbp_output_fn_;
int inference_t::edbp_max_iter_;

mpi_slam_t *mpi_base_t::mpi_ = 0;


void finalize() {
#ifdef USE_MPI
    mpi_base_t::mpi_->finalize_workers();
    delete mpi_base_t::mpi_;
#endif
}

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
    string tmp_path = "";
    vector<string> tracking_algorithms_str;

    float pa = 0.8;
    float po = 0.9;

    bool oreslam = false;
    int gtype = -1;
    int ptype = 0;
    int execution_length = 10;
    float map_threshold = .70;

    // inference algorithm
    string inference_algorithm = "bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=3)";
    //string inference_algorithm = "edbp(maxiter=2)";
    //string inference_algorithm = "jt(updates=HUGIN)";
    //string inference_algorithm = "cbp(updates=SEQRND,clamp=CLAMP_VAR,choose=CHOOSE_RANDOM,min_max_adj=10,bbp_props=,bbp_cfn=,recursion=REC_FIXED,tol=1e-3,rec_tol=1e-3,maxiter=100)";
    //string inference_algorithm = "lc(updates=SEQRND,cavity=FULL,logdomain=false,tol=1e-3,maxiter=100,maxtime=1,damping=.2)";
    //string inference_algorithm = "mr(updates=LINEAR,inits=RESPPROP,logdomain=false,tol=1e-3,maxiter=100,maxtime=1,damping=.2)";
    //string inference_algorithm = "hak(doubleloop=true,clusters=MIN,init=UNIFORM,tol=1e-3,maxiter=100,maxtime=1)";

#ifdef USE_MPI
    mpi_base_t::mpi_ = new mpi_slam_t(argc, argv);
#endif

    // parse arguments
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
        } else if( !strcmp(argv[0], "-g") || !strcmp(argv[0], "--gtype") ) {
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
        } else if( !strcmp(argv[0], "--map-threshold") ) {
            map_threshold = strtod(argv[1], 0);
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
        } else if( !strcmp(argv[0], "-o") || !strcmp(argv[0], "--ore-slam") ) {
            oreslam = true;
            --argc;
            ++argv;
        } else if( !strncmp(argv[0], "--tracker=", 10) ) {
            char *str = strdup(&argv[0][10]);
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
    srand48(seed);
    cout << "# seed=" << seed << endl;

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
    cellmap_t cellmap(nrows, ncols, nlabels, oreslam, pa, po);

    // set static members
    coord_t::ncols_ = ncols;
    base_particle_t::base_ = &cellmap;
    rbpf_slam2_particle_t::compute_edbp_factor_indices();
    inference_t::set_inference_algorithm(inference_algorithm, "BEL", tmp_path);

    // tracking algorithms
    vector<tracking_t<cellmap_t>*> tracking_algorithms;
    dai::PropertySet opts;
    int nparticles = 100;
    //int memory = 0;

    for( size_t i = 0; i < tracking_algorithms_str.size(); ++i ) {
        char *str = strdup(tracking_algorithms_str[i].c_str());
        string name = strtok(str, ":");
        char *token = strtok(0, ":");
        tracking_t<cellmap_t> *tracker = 0;
        if( name == "jt" ) {
#if 0
            memory = token != 0 ? atoi(token) : memory;
            pcbt_t<cellmap_t> *pcbt = new pcbt_t<cellmap_t>(name + "_" + to_string(memory), cellmap, memory);
            pcbt->set_algorithm_and_options("JT", opts("updates", string("HUGIN")));
            tracker = pcbt;
#endif
        } else if( name == "bp" ) {
#if 0
            memory = token != 0 ? atoi(token) : memory;
            pcbt_t<cellmap_t> *pcbt = new pcbt_t<cellmap_t>(name + "_" + to_string(memory), cellmap, memory);
            pcbt->set_algorithm_and_options("BP", opts("updates", string("SEQRND"))("logdomain", false)("tol", 1e-9)("maxiter", (size_t)10000));
            tracker = pcbt;
#endif
        } else if( name == "hak" ) {
#if 0
            memory = token != 0 ? atoi(token) : memory;
            pcbt_t<cellmap_t> *pcbt = new pcbt_t<cellmap_t>(name + "_" + to_string(memory), cellmap, memory);
            pcbt->set_algorithm_and_options("HAK", opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));
            tracker = pcbt;
#endif
        } else if( name == "ppcbt_jt" ) {
#if 0
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 1000);
            pcbt->set_algorithm_and_options("JT", opts("updates", string("HUGIN")));
            tracker = pcbt;
#endif
        } else if( name == "ppcbt_bp" ) {
#if 0
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 50);
            pcbt->set_algorithm_and_options("HAK", opts("doubleloop", true)("clusters", string("MIN"))("init", string("UNIFORM"))("tol", 1e-9)("maxiter", (size_t)10000)("maxtime", double(2)));
            tracker = pcbt;
#endif
        } else if( name == "ppcbt_hak" ) {
#if 0
            ppcbt2_t *pcbt = new ppcbt2_t(name + "_" + to_string(memory), cellmap, 50);
            pcbt->set_algorithm_and_options("BP", opts("updates", string("SEQRND"))("logdomain", false)("tol", 1e-9)("maxiter", (size_t)10000));
            tracker = pcbt;
#endif
        } else {
            // the tracking algorithm is a particle filter algorithm
            nparticles = token != 0 ? atoi(token) : nparticles;
            string full_name = name + "_" + to_string((long long)nparticles);

#ifdef USE_MPI
            // check that we have at least one worker for each particle
            if( 1 + nparticles > mpi_base_t::mpi_->nworkers_ ) {
                cout << "MPI: there are not enough workers for tracker '" << tracking_algorithms_str[i]
                     << "' (nworkers=" << mpi_base_t::mpi_->nworkers_ - 1
                     << "); continuing without this tracker"
                     << endl;
                continue;
            }
#endif

            if( name == "sis" ) {
                tracker = new SIS_t<sis_slam_particle_t, cellmap_t>(full_name, cellmap, nparticles);
            } else if( name == "mm_sir" ) {
                tracker = new motion_model_SIR_t<motion_model_sir_slam_particle_t, cellmap_t>(full_name, cellmap, nparticles);
            } else if( name == "opt_sir" ) {
                cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t> *cdf = new cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t>(cellmap);
                tracker = new optimal_SIR_t<optimal_sir_slam_particle_t, cellmap_t, cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t> >(full_name, cellmap, *cdf, nparticles);
            } else if( name == "mm_rbpf" ) {
                if( !oreslam )
                    tracker = new RBPF_t<motion_model_rbpf_slam_particle_t, cellmap_t>(full_name, cellmap, nparticles);
                else
                    tracker = new RBPF_t<motion_model_rbpf_slam2_particle_t, cellmap_t>(full_name, cellmap, nparticles);
            } else if( name == "opt_rbpf" ) {
                if( !oreslam )
                    tracker = new RBPF_t<optimal_rbpf_slam_particle_t, cellmap_t>(full_name, cellmap, nparticles);
                else
                    tracker = new RBPF_t<optimal_rbpf_slam2_particle_t, cellmap_t>(full_name, cellmap, nparticles);
            } else {
                cerr << "warning: unrecognized tracking algorithm '" << name << "'" << endl;
            }
        }
        free(str);
        if( tracker != 0 ) tracking_algorithms.push_back(tracker);
    }

    // check that there is something to do
    if( tracking_algorithms.empty() ) {
        cout << "warning: no tracker specified. Terminating..." << endl;
        finalize();
        return 0;
    }

    // print identity of trackers
    for( size_t i = 0; i < tracking_algorithms.size(); ++i )
        cout << "# tracker[" << i << "].id=\"" << tracking_algorithms[i]->id() << "\"" << endl;

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
    cellmap.set_labels(labels);

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
        //cellmap.compute_random_execution(0, execution_length, fixed_execution);
        cellmap.compute_covering_execution(0, fixed_execution, execution_length);
    }
    cout << "# fixed-execution[sz=" << fixed_execution.size() << "]=" << fixed_execution << endl;

    // run for the specified number of trials
    vector<repository_t> repos;
    for( int trial = 0; trial < ntrials; ++trial ) {
        float start_time = Utils::read_time_in_seconds();
        cellmap_t::execution_t output_execution;
        if( !fixed_execution.empty() )
            cellmap.run_execution(repos, fixed_execution, output_execution, tracking_algorithms, verbose);
        else
            cellmap.run_execution(repos, output_execution, cellmap.initial_loc_, nsteps, policy, tracking_algorithms, verbose);

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
        vector<pair<float, int> > map_values;
        cout << "# final(" << setw(size_longest_name) << "real" << "): map=";
        cellmap.print_labels(cout);
        cout << ", loc=" << coord_t(output_execution.back().loc_) << ":" << output_execution.back().loc_ << endl;
        for( size_t i = 0; i < tracking_algorithms.size(); ++i ) {
            cout << "# final(" << setw(size_longest_name) << tracking_algorithms[i]->name_ << "): map=[";
            for( int var = 0; var < nrows * ncols; ++var ) {
                tracking_algorithms[i]->MAP_on_var(repos[i], var, map_values, .1);
                assert(!map_values.empty());
                if( map_values[0].first < map_threshold ) {
                    cout << " *";
                } else {
                    if( map_values.size() == 1 ) {
                        cout << " " << map_values.back().second;
                    } else {
#if 1
                        cout << " +";
#else
                        cout << " {";
                        for( size_t k = 0; k < map_values.size(); ++k ) {
                            cout << map_values[k].second;
                            if( k < map_values.size() - 1 ) cout << ",";
                        }
                        cout << "}";
#endif
                    }
                }
                if( (var + 1 < nrows * ncols) && (((var + 1) % ncols) == 0) )
                    cout << " |";
            }

            cout << "], loc=";
            tracking_algorithms[i]->MAP_on_var(repos[i], nrows * ncols, map_values, .1);
            assert(!map_values.empty());
            if( map_values.size() == 1 ) {
                cout << coord_t(map_values.back().second) << ":" << map_values.back().second << endl;
            } else {
                cout << " {";
                for( size_t k = 0; k < map_values.size(); ++k ) {
                    cout << coord_t(map_values[k].second) << ":" << map_values[k].second;
                    if( k < map_values.size() - 1 ) cout << ",";
                }
                cout << "}" << endl;
            }
        }
        cout << "# '*' means more than one label in MAP for given position" << endl;


        if( !tracking_algorithms.empty() ) {
            const tracking_t<cellmap_t> &tracker = *tracking_algorithms.back();

            cout << "library(ggplot2)" << endl
                 << "library(reshape)" << endl
                 << "library(zoo)" << endl
                 << "#library(gtable)" << endl
                 << "library(grid)" << endl
                 << "library(gridExtra)" << endl;

            cout << endl
                 << "pdf(\"plot.pdf\", width = 12, height = 10)" << endl;

            // useful R functions
            cout << "define_region <- function(row, col) { viewport(layout.pos.row = row, layout.pos.col = col); }"
                 << endl
                 << "get_legend <- function(myggplot) { tmp <- ggplot_gtable(ggplot_build(myggplot)); leg <- which(sapply(tmp$grobs, function(x) x$name) == \"guide-box\"); legend <- tmp$grobs[[leg]]; return(legend); }"
                 << endl;

            cout << endl;

            // color themes from http://colorbrewer2.org
            cout << "color_theme1 <- c(\"#543005\",\"#8c510a\",\"#bf812d\",\"#dfc27d\",\"#f6e8c3\",\"#f5f5f5\",\"#c7eae5\",\"#80cdc1\",\"#35978f\",\"#01665e\",\"#003c30\")" << endl;
            cout << "color_theme2 <- c(\"#276419\",\"#4d9221\",\"#7fbc41\",\"#b8e186\",\"#e6f5d0\",\"#f7f7f7\",\"#fde0ef\",\"#f1b6da\",\"#de77ae\",\"#c51b7d\",\"#8e0152\")" << endl;
            cout << "color_theme3 <- c(\"#00441b\",\"#1b7837\",\"#5aae61\",\"#a6dba0\",\"#d9f0d3\",\"#f7f7f7\",\"#e7d4e8\",\"#c2a5cf\",\"#9970ab\",\"#762a83\",\"#40004b\")" << endl;
            cout << "color_theme4 <- c(\"#7f3b08\",\"#b35806\",\"#e08214\",\"#fdb863\",\"#fee0b6\",\"#f7f7f7\",\"#d8daeb\",\"#b2abd2\",\"#8073ac\",\"#542788\",\"#2d004b\")" << endl;
            cout << "color_theme5 <- c(\"#053061\",\"#2166ac\",\"#4393c3\",\"#92c5de\",\"#d1e5f0\",\"#f7f7f7\",\"#fddbc7\",\"#f4a582\",\"#d6604d\",\"#b2182b\",\"#67001f\")" << endl;
            cout << "color_theme6 <- c(\"#67001f\",\"#b2182b\",\"#d6604d\",\"#f4a582\",\"#fddbc7\",\"#ffffff\",\"#e0e0e0\",\"#bababa\",\"#878787\",\"#4d4d4d\",\"#1a1a1a\")" << endl;
            cout << "color_theme7 <- c(\"#313695\",\"#4575b4\",\"#74add1\",\"#abd9e9\",\"#e0f3f8\",\"#e8e8e8\",\"#fee090\",\"#fdae61\",\"#f46d43\",\"#d73027\",\"#a50026\")" << endl;
            cout << "color_theme8 <- c(\"#006837\",\"#1a9850\",\"#66bd63\",\"#a6d96a\",\"#d9ef8b\",\"#b8b8b8\",\"#fee08b\",\"#fdae61\",\"#f46d43\",\"#d73027\",\"#a50026\")" << endl;
            cout << "color_theme9 <- c(\"#5e4fa2\",\"#3288bd\",\"#66c2a5\",\"#abdda4\",\"#e6f598\",\"#ffffbf\",\"#fee08b\",\"#fdae61\",\"#f46d43\",\"#d53e4f\",\"#9e0142\")" << endl;

            cout << endl;

            // prepare colors and labels for legend: 10 levels
            cout << "quantile_range <- c(0, .01, .05, .1, .25, .45, .55, .75, .9, .95, .99, 1)" << endl;
            cout << "color_palette <- colorRampPalette(color_theme2)(length(quantile_range) - 1)" << endl;
            cout << "label_text <- rollapply(round(quantile_range, 2), width = 2, by = 1, FUN = function(i) paste(i, collapse = \"-\"))" << endl;

            cout << endl;

            // define minimal theme
            cout << "minimal_theme <- theme(" << endl
                 << "    plot.background = element_blank()," << endl
                 << "    panel.grid.minor = element_blank()," << endl
                 << "    panel.grid.major = element_blank()," << endl
                 << "    panel.background = element_blank()," << endl
                 << "    panel.border = element_blank()," << endl
                 << "    axis.line = element_blank()," << endl
                 << "    axis.ticks = element_blank()," << endl
                 << "    axis.text.x = element_blank()," << endl
                 << "    axis.text.y = element_blank()," << endl
                 << "    axis.title.x = element_blank()," << endl
                 << "    axis.title.y = element_blank()," << endl
                 << "    plot.margin = unit(c(-.25, -.5, -1.25, -.5), \"lines\")," << endl
                 << "    legend.title = element_blank()" << endl
                 << ")" << endl;

            cout << endl;

            // define time steps for analysis
            set<int> focus;
            int nsteps = int(repos.back().size());
            for( int t = 0; t < nsteps; ++t ) {
                if( t == int(.25 * nsteps) ) // raw data at 25% time of total steps
                    focus.insert(t);
                else if( t == int(.5 * nsteps) ) // raw data at 50% time of total steps
                    focus.insert(t);
                else if( t == int(.75 * nsteps) ) // raw data at 75% time of total steps
                    focus.insert(t);
                else if( t == nsteps - 1 ) // raw data at end of time
                    focus.insert(t);
            }

            // extract marginals and maps for selected time steps
            cout << "time_steps <- list()" << endl;
            cout << "raw_data <- list()" << endl;
            for( int t = 0; t < nsteps; ++t ) {
                if( focus.find(t) != focus.end() ) {
                    cout << "time_steps <- append(time_steps, list(" << t << "))" << endl;

                    // raw data for ore field
                    cout << "raw_data <- append(raw_data, list(matrix(c(";
                    for( int var = 0; var < nrows * ncols; ++var ) {
                        const cell_t &cell = cellmap.cells_[var];
                        cout << (cell.label_ ? 1 : 0);
                        if( 1 + var < nrows * ncols ) cout << ", ";
                    }
                    cout << "), ncol=" << ncols << ", byrow=T)))" << endl;

                    // raw data for marginals
                    cout << "raw_data <- append(raw_data, list(matrix(c(";
                    for( int var = 0; var < nrows * ncols; ++var ) {
                        const float *marginal = &repos.back()[t][cellmap.variable_offset(var)];
                        cout << marginal[1];
                        if( var + 1 < nrows * ncols ) cout << ", ";
                        assert(fabs(marginal[0] + marginal[1] - 1.0) < EPSILON);
                    }
                    cout << "), ncol=" << ncols << ", byrow=T)))" << endl;

                    // raw data for maps on marginals
                    cout << "raw_data <- append(raw_data, list(matrix(c(";
                    for( int var = 0; var < nrows * ncols; ++var ) {
                        vector<pair<float, int> > map_values;
                        tracker.MAP_on_var(repos.back(), t, var, map_values, .1);
                        assert(!map_values.empty());
                        if( map_values[0].first < map_threshold )
                            cout << .50;
                        else
                            cout << (map_values.size() == 1 ? map_values[0].second : .50);
                        if( 1 + var < nrows * ncols ) cout << ", ";
                    }
                    cout << "), ncol=" << ncols << ", byrow=T)))" << endl;
                }
            }
            cout << endl;

            // apply quantile discretization to field, marginals and maps
            cout << "mod_data <- lapply(raw_data, function(d) { m <- matrix(findInterval(d, quantile_range, all.inside = TRUE), nrow = nrow(d)); m; })" << endl;

            cout << endl;

            // generate plots and plots without legends
            cout << "plots <- lapply(mod_data, function(m) { p <- ggplot(melt(m), aes(x = X2, y = X1, fill = factor(value, levels = 1:(length(quantile_range) - 1)))) + geom_tile(color = \"black\") + scale_fill_manual(values = color_palette, name = \"\", labels = label_text, drop = FALSE) + minimal_theme; p })" << endl;
            cout << "plots_nl <- lapply(plots, function(p) { pnl <- p + theme(legend.position = \"none\"); pnl })" << endl;
            cout << "plot_legend <- get_legend(plots[[1]])" << endl;

            cout << endl;

            // calculate # errors in maps
            cout << "map_threshold <- " << map_threshold << endl;
            cout << "n_rows <- " << nrows << endl;
            cout << "n_cols <- " << ncols << endl;
            cout << "n_time_steps <- " << focus.size() << endl;
            cout << "equals_in_map <- sapply(seq(1, 3 * n_time_steps, 3), function(i) { sum(raw_data[[i+2]] == raw_data[[i]]) })" << endl;
            cout << "unknowns_in_map <- sapply(seq(1, 3 * n_time_steps, 3), function(i) { sum(raw_data[[i+2]] == 0.5) })" << endl;
            cout << "errors_in_map <- n_rows * n_cols - equals_in_map - unknowns_in_map" << endl;

            cout << endl;

            // put plots together using viewports and display them
            //cout << "grid.newpage()" << endl
            cout << "pushViewport(viewport(layout = grid.layout(2 + n_time_steps, 5, heights = unit(c(1.75, rep(4, n_time_steps), .5), rep(\"null\", 2 + n_time_steps)), widths = unit(rep(4, 5), rep(\"null\", 5)))))" << endl
                 << "sapply(seq_along(plots_nl), function(i) { print(plots_nl[[i]], vp = define_region(2 + ((i - 1) %/% 3), 2 + ((i - 1) %% 3))); i })" << endl
                 << "grid.text(\"ore field\", vp = define_region(2 + n_time_steps, 2))" << endl
                 << "grid.text(\"marginals\", vp = define_region(2 + n_time_steps, 3))" << endl
                 << "grid.text(\"map on marginals\", vp = define_region(2 + n_time_steps, 4))" << endl
                 << "grid.text(paste(n_rows, \"x\", n_cols, \" grid problem\\n"
                 << tracker.id()
                 << "\\n"
                 << inference_algorithm
                 << "\", sep=\"\"), vp = viewport(layout.pos.row = 1, layout.pos.col = 1:5))" << endl
                 << "sapply(seq_along(time_steps), function(i) { grid.text(paste(\"After\", time_steps[[i]], \"steps\\n#error(s) in MAP =\", errors_in_map[[i]], sep=\" \"), vp = define_region(1 + i, 1)); i })" << endl
                 << "pushViewport(viewport(just = c(\"center\", \"center\"), layout.pos.row = 2:(1 + n_time_steps), layout.pos.col = 5))" << endl
                 << "grid.draw(plot_legend)" << endl
                 << "dev.off()" << endl;

            // generate R plots
#if 0
            cout << "library(\"reshape2\");" << endl << "library(\"ggplot2\");" << endl;
            for( size_t i = 0; i < tracking_algorithms.size(); ++i )
                cellmap.generate_R_plot(cout, *tracking_algorithms[i], repos[i]);
#endif
        }

        float elapsed_time = Utils::read_time_in_seconds() - start_time;
        cout << "# elapsed time=" << elapsed_time << endl;
    }

    finalize();
    return 0;
}

