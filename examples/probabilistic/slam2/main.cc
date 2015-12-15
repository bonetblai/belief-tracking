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
#include "generate_plot.h"
#include "stats.h"

#include "sis.h"
#include "motion_model_sir.h"
#include "optimal_sir.h"
#include "rbpf.h"

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

#ifdef USE_MPI // static members for load balancing
template <typename PTYPE, typename BASE> int SIR_t<PTYPE, BASE>::mpi_machine_for_master_ = 0;
template <typename PTYPE, typename BASE> vector<vector<int> > SIR_t<PTYPE, BASE>::mpi_fixed_budget_;
#endif


void set_labels_and_execution(cellmap_t &cellmap, int ptype, int num_covering_loops, cellmap_t::execution_t &execution) {
    // set labels
    vector<int> labels;
#if 0
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
    }
#endif
    cellmap.sample_labels(labels);
    cellmap.set_labels(labels);

    // set execution
    execution.clear();
#if 0
    if( ptype == 0 ) {
        if( gtype < 2 ) {
            execution.push_back(cellmap_t::execution_step_t(0, 0, -1));
            execution.push_back(cellmap_t::execution_step_t(1, 1, cellmap_t::right));
            execution.push_back(cellmap_t::execution_step_t(2, 0, cellmap_t::right));
            execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::right));
            execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::right));
            execution.push_back(cellmap_t::execution_step_t(4, 0, cellmap_t::right));
            execution.push_back(cellmap_t::execution_step_t(5, 1, cellmap_t::right));
            execution.push_back(cellmap_t::execution_step_t(6, 0, cellmap_t::right));
            execution.push_back(cellmap_t::execution_step_t(7, 1, cellmap_t::right));
            execution.push_back(cellmap_t::execution_step_t(6, 0, cellmap_t::left));
            execution.push_back(cellmap_t::execution_step_t(5, 1, cellmap_t::left));
            execution.push_back(cellmap_t::execution_step_t(4, 0, cellmap_t::left));
            execution.push_back(cellmap_t::execution_step_t(3, 1, cellmap_t::left));
            execution.push_back(cellmap_t::execution_step_t(2, 0, cellmap_t::left));
            execution.push_back(cellmap_t::execution_step_t(1, 1, cellmap_t::left));
            execution.push_back(cellmap_t::execution_step_t(0, 0, cellmap_t::left));
        }
    }
#endif
    if( ptype == 1 )
        cellmap.compute_covering_execution(0, execution, num_covering_loops);
    cout << "# fixed-execution: sz=" << execution.size() << endl;
    //cout << "# fixed-execution[sz=" << execution.size() << "]=" << execution << endl;
}

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
    vector<string> tracker_strings;

    float pa = 0.9; // default value in K. P. Murphy's paper
    float po = 0.8; // default value in K. P. Murphy's paper

    bool oreslam = false;
    int gtype = -1;
    int ptype = 0;
    int num_covering_loops = 10;
    float map_threshold = .55;
    float map_epsilon = .05;

    float epsilon = .001;
    float discount = .95;

    bool force_resampling = false;
    bool do_stochastic_universal_sampling = false;
    bool R_plot = false;

    // inference algorithm
    string inference_algorithm = "bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=3)";
    //string inference_algorithm = "edbp(maxiter=2)";
    //string inference_algorithm = "jt(updates=HUGIN)";
    //string inference_algorithm = "cbp(updates=SEQRND,clamp=CLAMP_VAR,choose=CHOOSE_RANDOM,min_max_adj=10,bbp_props=,bbp_cfn=,recursion=REC_FIXED,tol=1e-3,rec_tol=1e-3,maxiter=100)";
    //string inference_algorithm = "lc(updates=SEQRND,cavity=FULL,logdomain=false,tol=1e-3,maxiter=100,maxtime=1,damping=.2)";
    //string inference_algorithm = "mr(updates=LINEAR,inits=RESPPROP,logdomain=false,tol=1e-3,maxiter=100,maxtime=1,damping=.2)";
    //string inference_algorithm = "hak(doubleloop=true,clusters=MIN,init=UNIFORM,tol=1e-3,maxiter=100,maxtime=1)";

    // start global timer
    float start_time = Utils::read_time_in_seconds();

#ifdef USE_MPI
    mpi_base_t::mpi_ = new mpi_slam_t(argc, argv);

    // get worker distribution and budget
    int mpi_machine_for_master;
    map<string, int> mpi_processor_name_map;
    vector<vector<int> > mpi_fixed_budget;
    mpi_base_t::mpi_->get_worker_distribution(mpi_processor_name_map, mpi_fixed_budget, mpi_machine_for_master);

    cout << "MPI: " << mpi_base_t::mpi_->nworkers_ << " process(es) running in " << mpi_processor_name_map.size() << " processor(s):" << endl;
    for( map<string, int>::const_iterator it = mpi_processor_name_map.begin(); it != mpi_processor_name_map.end(); ++it )
        cout << "MPI:     -> " << it->first << " (" << it->second << ")" << endl;

    // set static elements for load balancing
    SIR_t<motion_model_sir_slam_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
    SIR_t<optimal_sir_slam_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
    SIR_t<motion_model_rbpf_slam_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
    SIR_t<optimal_rbpf_slam_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
    SIR_t<motion_model_rbpf_slam2_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
    SIR_t<optimal_rbpf_slam2_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;

    SIR_t<motion_model_sir_slam_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<optimal_sir_slam_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<motion_model_rbpf_slam_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<optimal_rbpf_slam_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<motion_model_rbpf_slam2_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<optimal_rbpf_slam2_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
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
        } else if( !strcmp(argv[0], "--map-epsilon") ) {
            map_epsilon = strtod(argv[1], 0);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "--map-threshold") ) {
            map_threshold = strtod(argv[1], 0);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-p") || !strcmp(argv[0], "--policy") ) {
            ptype = atoi(argv[1]);
            int n = atoi(argv[2]);
            if( ptype < 2 ) {
                num_covering_loops = n;
            } else if( ptype == 2 ) {
                nsteps = n;
            } else if( ptype == 3 ) {
                nsteps = n;
            }
            argc -= 3;
            argv += 3;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "--fr") || !strcmp(argv[0], "--force-resampling") ) {
            force_resampling = true;
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--sus") || !strcmp(argv[0], "--stochastic-universal-sampling") ) {
            do_stochastic_universal_sampling = true;
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--ore-slam") ) {
            oreslam = true;
            --argc;
            ++argv;
        } else if( !strncmp(argv[0], "--tracker=", 10) ) {
            char *str = strdup(&argv[0][10]);
            char *token = strtok(str, ",");
            while( token != 0 ) {
                tracker_strings.push_back(token);
                token = strtok(0, ",");
            }
            free(str);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--plot") || !strcmp(argv[0], "--generate-plot-R") ) {
            R_plot = true;
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
    vector<tracking_t<cellmap_t>*> trackers;
    dai::PropertySet opts;
    int nparticles = 100;
    //int memory = 0;

    for( size_t i = 0; i < tracker_strings.size(); ++i ) {
        char *str = strdup(tracker_strings[i].c_str());
        string name = strtok(str, ":");
        char *token = strtok(0, ":");
        tracking_t<cellmap_t> *tracker = 0;

        // the tracking algorithm is a particle filter algorithm
        nparticles = token != 0 ? atoi(token) : nparticles;
        string full_name = name + "_" + to_string((long long)nparticles);

#ifdef USE_MPI
        // check that we have at least one worker for each particle
        if( 1 + nparticles > mpi_base_t::mpi_->nworkers_ ) {
            cout << "MPI: there are not enough workers for tracker '" << tracker_strings[i]
                 << "' (nworkers=" << mpi_base_t::mpi_->nworkers_ - 1
                 << "); continuing without this tracker"
                 << endl;
            continue;
        }
#endif

        if( name == "sis" ) {
            tracker = new SIS_t<sis_slam_particle_t, cellmap_t>(full_name, cellmap, nparticles);
        } else if( name == "mm_sir" ) {
            tracker = new motion_model_SIR_t<motion_model_sir_slam_particle_t,
                                             cellmap_t>(full_name,
                                                        cellmap,
                                                        nparticles,
                                                        force_resampling,
                                                        do_stochastic_universal_sampling);
        } else if( name == "opt_sir" ) {
            cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t> *cdf =
              new cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t>(cellmap);
            tracker = new optimal_SIR_t<optimal_sir_slam_particle_t,
                                        cellmap_t,
                                        cdf_for_optimal_sir_t<optimal_sir_slam_particle_t,
                                                              cellmap_t> >(full_name,
                                                                           cellmap,
                                                                           *cdf,
                                                                           nparticles,
                                                                           force_resampling,
                                                                           do_stochastic_universal_sampling);
        } else if( name == "mm_rbpf" ) {
            if( !oreslam ) {
                tracker = new RBPF_t<motion_model_rbpf_slam_particle_t,
                                     cellmap_t>(full_name,
                                                cellmap,
                                                nparticles,
                                                force_resampling,
                                                do_stochastic_universal_sampling);
            } else {
                tracker = new RBPF_t<motion_model_rbpf_slam2_particle_t,
                                     cellmap_t>(full_name,
                                                cellmap,
                                                nparticles,
                                                force_resampling,
                                                do_stochastic_universal_sampling);
            }
        } else if( name == "opt_rbpf" ) {
            if( !oreslam ) {
                tracker = new RBPF_t<optimal_rbpf_slam_particle_t,
                                     cellmap_t>(full_name,
                                                cellmap,
                                                nparticles,
                                                force_resampling,
                                                do_stochastic_universal_sampling);
            } else {
                tracker = new RBPF_t<optimal_rbpf_slam2_particle_t,
                                     cellmap_t>(full_name,
                                                cellmap,
                                                nparticles,
                                                force_resampling,
                                                do_stochastic_universal_sampling);
            }
        } else {
            cerr << "warning: unrecognized tracking algorithm '" << name << "'" << endl;
        }
        free(str);
        if( tracker != 0 ) trackers.push_back(tracker);
    }

    // check that there is something to do
    if( trackers.empty() ) {
        cout << "warning: no tracker specified. Terminating..." << endl;
        finalize();
        return 0;
    }

    // print identity of trackers
    for( size_t i = 0; i < trackers.size(); ++i )
        cout << "# tracker[" << i << "].id=\"" << trackers[i]->id() << "\"" << endl;

    // compute longest name
    size_t size_longest_name = 0;
    for( size_t i = 0; i < trackers.size(); ++i )
        size_longest_name = size_longest_name > trackers[i]->name_.size() ? size_longest_name : trackers[i]->name_.size();

    // action selection
    vector<action_selection_t<cellmap_t>*> policy;
    policy.push_back(0);
    policy.push_back(new random_slam_policy_t(cellmap));
    policy.push_back(new exploration_slam_policy_t(cellmap, map_epsilon));
    policy.push_back(new murphy_nips99_slam_policy_t(cellmap, discount, epsilon));

    // set initial loc
    cellmap.initial_loc_ = 0;

    // set common elements for R plots
    if( R_plot )
        generate_R_plot_commons();

    // run for the specified number of trials and collect statistics
    cellmap_t::execution_t fixed_execution;
    vector<Utils::stat_t> stats_unknown(trackers.size());
    vector<Utils::stat_t> stats_error(trackers.size());
    for( int trial = 0; trial < ntrials; ++trial ) {
        cout << "# trial = " << trial << endl;

        // set labels and fixed execution (if appropriate)
        set_labels_and_execution(cellmap, ptype, num_covering_loops, fixed_execution);

        // run execution
        vector<repository_t> repos;
        cellmap_t::execution_t output_execution;
        if( !fixed_execution.empty() )
            cellmap.run_execution(repos, fixed_execution, output_execution, trackers, verbose);
        else
            cellmap.run_execution(repos, output_execution, cellmap.initial_loc_, nsteps, *policy[ptype], trackers, verbose);
        cout << "# output-execution: sz=" << output_execution.size() << endl;
        //cout << "# output-execution[sz=" << output_execution.size() << "]=" << output_execution << endl;
        //trackers[0]->print(cout);

        // calculate final marginals
        for( size_t i = 0; i < trackers.size(); ++i )
            trackers[i]->calculate_marginals();

#if 0
        // compute and print scores
        vector<cellmap_t::score_t> scores;
        cellmap.compute_scores(output_execution, trackers, scores);
        for( size_t i = 0; i < scores.size(); ++i ) {
            cout << "# score(" << setw(size_longest_name) << trackers[i]->name_ << "): score=" << scores[i] << endl;
        }
#endif

        // print final (tracked) map and location
        vector<pair<float, int> > map_values;
        cout << "# final(" << setw(size_longest_name) << "real" << "): map=";
        cellmap.print_labels(cout);
        cout << ", loc=" << coord_t(output_execution.back().loc_) << ":" << output_execution.back().loc_ << endl;

        // do plots and print final MAP for each tracker
        for( size_t i = 0; i < trackers.size(); ++i ) {
            const tracking_t<cellmap_t> &tracker = *trackers[i];

            // do plots
            if( R_plot ) {
                generate_R_plot(string("plot_") + to_string(trial) + "_" + to_string(i) + ".pdf",
                                tracker,
                                repos[i],
                                cellmap,
                                inference_algorithm,
                                map_epsilon,
                                map_threshold);

#if 0
                cout << "library(\"reshape2\");" << endl << "library(\"ggplot2\");" << endl;
                for( size_t i = 0; i < trackers.size(); ++i )
                    cellmap.generate_R_plot(cout, *trackers[i], repos[i]);
#endif
            }

            // print map and stats
            int unknowns = 0;
            int errors = 0;
            cout << "# final(" << setw(size_longest_name) << tracker.name_ << "): map=[";
            for( int loc = 0; loc < nrows * ncols; ++loc ) {
                tracker.MAP_on_var(repos[i], loc, map_values, map_epsilon);
                assert(!map_values.empty());
                if( (map_values[0].first < map_threshold) || (map_values.size() > 1) ) {
                    cout << " *";
                    ++unknowns;
                } else {
                    cout << " " << map_values.back().second;
                    errors += map_values.back().second == cellmap.cells_[loc].label_ ? 0 : 1;
                }
                if( (loc + 1 < nrows * ncols) && (((loc + 1) % ncols) == 0) )
                    cout << " |";
            }

            cout << "], loc=";
            tracker.MAP_on_var(repos[i], nrows * ncols, map_values, map_epsilon);
            assert(!map_values.empty());
            if( map_values.size() == 1 ) {
                cout << coord_t(map_values.back().second)
                     << ":" << map_values.back().second
                     << ":" << setprecision(4) << map_values.back().first;
            } else {
                cout << "{";
                for( size_t k = 0; k < map_values.size(); ++k ) {
                    cout << coord_t(map_values[k].second) << ":" << map_values[k].second << ":" << setprecision(4) << map_values[k].first;
                    if( k < map_values.size() - 1 ) cout << ",";
                }
                cout << "}";
            }

            cout << ", #unknowns=" << unknowns
                 << ", #errors=" << errors
                 << ", elapsed-time=" << tracker.elapsed_time()
                 << endl;
            stats_unknown[i].add(unknowns);
            stats_error[i].add(errors);
        }

        // clean trackers and repos
        assert(trackers.size() == repos.size());
        for( int i = 0; i < int(trackers.size()); ++i ) {
            tracking_t<cellmap_t> &tracker = *trackers[i];
            repository_t &repo = repos[i];
            tracker.clear();
            repo.clear_marginals();
        }
    }

    // print average statistics
    for( size_t i = 0; i < trackers.size(); ++i ) {
        const tracking_t<cellmap_t> &tracker = *trackers[i];
        cout << "# stats(" << setw(size_longest_name) << tracker.name_ << "):"
             << " trials=" << stats_unknown[i].n()
             << ", total-elapsed-time=" << tracker.total_elapsed_time()
             << ", avg-unknowns-per-trial=" << stats_unknown[i].mean() << " (" << stats_unknown[i].confidence(.95) << ")"
             << ", avg-errors-per-trial=" << stats_error[i].mean() << " (" << stats_error[i].confidence(.95) << ")"
             << ", avg-elapsed-time-per-trial=" << tracker.total_elapsed_time() / ntrials
             << ", avg-elapsed-time-per-step=" << tracker.total_elapsed_time() / tracker.total_num_steps()
             << endl;
    }

    // stop timer
    float elapsed_time = Utils::read_time_in_seconds() - start_time;
    cout << "# total time = " << elapsed_time << endl;

    finalize();
    return 0;
}

