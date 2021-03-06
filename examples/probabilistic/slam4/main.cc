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
#include <fstream>
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

#include "decomposable.h"
#include "sis.h"
#include "motion_model_sir.h"
#include "optimal_sir.h"
#include "rbpf.h"

#include "slam_particles.h"
#include "mine_mapping_particles.h"
#if 0
#include "mine_mapping_v2_particles.h"
#endif
#include "corridor_slam_particles.h"

using namespace std;


// static members
int coord_t::ncols_ = 0;
const cellmap_t *base_particle_t::base_ = 0;
template <> const cellmap_t *Decomposable::decomposable_t<cellmap_t>::static_base_ = 0;


// static members for inference algorithms
string Inference::inference_t::edbp_factors_fn_;
string Inference::inference_t::edbp_evid_fn_;
string Inference::inference_t::edbp_output_fn_;
vector<vector<int> > Inference::edbp_t::edbp_factor_indices_;

// static member for dai cache
int dai::cache_t::num_locs_ = 0;
vector<dai::Var> dai::cache_t::variables_;
vector<dai::VarSet> dai::cache_t::varsets_;
vector<vector<map<dai::Var, size_t>*> > dai::cache_t::state_cache_;

int dai::cache_with_location_t::num_locs_ = 0;
vector<dai::Var> dai::cache_with_location_t::variables_;
vector<dai::VarSet> dai::cache_with_location_t::varsets_;
vector<vector<map<dai::Var, size_t>*> > dai::cache_with_location_t::state_cache_;

// static member for SLAM cache
vector<vector<int> > SLAM::cache_t::slabels_;
vector<vector<int> > SLAM::cache_with_location_t::slabels_;
vector<vector<int> > SLAM::cache_with_location_t::patched_values_;

// static members for kappa handling
float kappa_t::epsilon_ = 0;
vector<float> kappa_t::powers_;

// static members for mine-mapping particles
vector<unsigned*> MineMapping::cache_t::compatible_values_;
CSP::constraint_digraph_t MineMapping::kappa_arc_consistency_t::cg_;

#if 0 // V2
// static members for mine-mapping v2 particles
vector<unsigned*> MineMappingV2::cache_t::compatible_values_;
vector<dai::VarSet> MineMappingV2::cache_t::external_vars_v2_;
vector<vector<int> > MineMappingV2::cache_t::external_vars_map_v2_;
CSP::constraint_digraph_t MineMappingV2::kappa_arc_consistency_t::cg_;
#endif

// static members for 1d-slam particles
vector<unsigned> Corridor::cache_t::compatible_values_;
CSP::constraint_digraph_t Corridor::kappa_arc_consistency_t::cg_;

// static members for MPI
mpi_slam_t *mpi_base_t::mpi_ = 0;

#ifdef USE_MPI // static members for load balancing
template <typename PTYPE, typename BASE> int SIR_t<PTYPE, BASE>::mpi_machine_for_master_ = 0;
template <typename PTYPE, typename BASE> vector<vector<int> > SIR_t<PTYPE, BASE>::mpi_fixed_budget_;
#endif


bool read_labels_and_execution(ifstream &ifs, cellmap_t &cellmap, cellmap_t::execution_t &execution) {
    bool rv = cellmap.read_labels(ifs) && execution.read(ifs);
    cout << "# read_labels_and_execution: execution-size=" << execution.size() << endl;
    return rv;
}

void write_labels_and_execution(ofstream &ofs, const cellmap_t &cellmap, const cellmap_t::execution_t &execution) {
    cellmap.dump_labels(ofs);
    ofs << " ";
    execution.dump(ofs);
}

void set_labels_and_execution(cellmap_t &cellmap, int ptype, int num_covering_loops, cellmap_t::execution_t &execution) {
    // set labels
    vector<int> labels;
    cellmap.sample_labels(labels);
    cellmap.set_labels(labels);

    // set execution
    execution.clear();
    if( ptype == 1 )
        cellmap.compute_covering_execution(0, execution, num_covering_loops);
    cout << "# set_labels_and_execution: execution-size=" << execution.size() << endl;
    //cout << "# set_labels_and_execution: execution[sz=" << execution.size() << "]=" << execution << endl;
}

void finalize() {
#ifdef USE_MPI
    mpi_base_t::mpi_->finalize_workers();
    delete mpi_base_t::mpi_;
    mpi_base_t::mpi_ = 0;
#endif
}

void usage(ostream &os) {
    os << endl
       << "Usage: slam [--task {color | mine-mapping-peaked | mine-mapping-non-peaked | 1d} (default: color)]" << endl
       << "            [{-c | --ncols} <int> (default: 1)]" << endl
       << "            [{-r | --nrows} <int> (default: 1)]" << endl
       << "            [{-l | --nlabels} <int> (default: 2)]" << endl
       << "            [{-n | --nsteps} <int> (default: 10)]" << endl
       << "            [{-t | --ntrials} <int> (default: 1)]" << endl
       << "            [--pa <p> (default: 0.9)]" << endl
       << "            [--po <p> (default: 0.8)]" << endl
       << "            [--kappa <p> (default: 0.1)]" << endl
       << "            [--map-epsilon <p> (default: 0.05)]" << endl
       << "            [--map-threshold <p> (default: 0.55)]" << endl
       << "            [{-p | --policy} <0=null,1=random,2=exploration,3=muprhy> <n> (default: 0 10)]" << endl
       << "            [--read-executions <file>]" << endl
       << "            [--save-execution-and-exit <file>]" << endl
       << "            [--tracker <tracker-spec>]" << endl
       << "            [{--plot | --generate-plot-R}]" << endl
       << "            [--tmp-path <path> (default: "")]" << endl
       << "            [{-s | --seed} <int> (default: 0)]" << endl
       << "            [{-v | --verbose}]" << endl
       << "            [{-? | --help}]" << endl
       << endl;
}

int main(int argc, const char **argv) {
    cout << "args:";
    for( int i = 0; i < argc; ++i )
        cout << " " << argv[i];
    cout << endl;

    int ntrials = 1;
    int nrows = 1;
    int ncols = 1;
    int nlabels = 2;
    int nsteps = 10;
    int seed = 0;
    bool verbose = false;
    string tmp_path = "";
    vector<string> tracker_strings;

    float pa = 0.9; // default value in K. P. Murphy's paper
    float po = 0.8; // default value in K. P. Murphy's paper
    float epsilon_for_kappa = 0.1;

    cellmap_t::slam_type_t slam_type = cellmap_t::COLOR;

    int ptype = 0;
    int num_covering_loops = 10;
    float map_threshold = .55;
    float map_epsilon = .05;

    float epsilon = .001;
    float discount = .95;

    bool R_plot = false;

    // input/output streams for saving/reading executions
    string filename_execution;
    bool read_execution = false;
    bool save_execution_and_exit = false;
    ifstream *ifs_execution = 0;
    ofstream *ofs_execution = 0;

#if 0
    // inference algorithm
    string inference_algorithm = "bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=3)";
    //string inference_algorithm = "edbp(maxiter=2)";
    //string inference_algorithm = "jt(updates=HUGIN)";
    //string inference_algorithm = "cbp(updates=SEQRND,clamp=CLAMP_VAR,choose=CHOOSE_RANDOM,min_max_adj=10,bbp_props=,bbp_cfn=,recursion=REC_FIXED,tol=1e-3,rec_tol=1e-3,maxiter=100)";
    //string inference_algorithm = "lc(updates=SEQRND,cavity=FULL,logdomain=false,tol=1e-3,maxiter=100,maxtime=1,damping=.2)";
    //string inference_algorithm = "mr(updates=LINEAR,inits=RESPPROP,logdomain=false,tol=1e-3,maxiter=100,maxtime=1,damping=.2)";
    //string inference_algorithm = "hak(doubleloop=true,clusters=MIN,init=UNIFORM,tol=1e-3,maxiter=100,maxtime=1)";
#endif

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
    SIR_t<MineMapping::motion_model_rbpf_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
    SIR_t<MineMapping::optimal_rbpf_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
#if 0 // V2
    SIR_t<MineMappingV2::motion_model_rbpf_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
    SIR_t<MineMappingV2::optimal_rbpf_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
#endif
    SIR_t<Corridor::motion_model_rbpf_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;
    SIR_t<Corridor::optimal_rbpf_particle_t, cellmap_t>::mpi_machine_for_master_ = mpi_machine_for_master;

    SIR_t<motion_model_sir_slam_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<optimal_sir_slam_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<motion_model_rbpf_slam_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<optimal_rbpf_slam_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<MineMapping::motion_model_rbpf_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<MineMapping::optimal_rbpf_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
#if 0 // V2
    SIR_t<MineMappingV2::motion_model_rbpf_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<MineMappingV2::optimal_rbpf_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
#endif
    SIR_t<Corridor::motion_model_rbpf_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
    SIR_t<Corridor::optimal_rbpf_particle_t, cellmap_t>::mpi_fixed_budget_ = mpi_fixed_budget;
#endif

    // parse arguments
    for( --argc, ++argv; (argc > 0) && (**argv == '-'); --argc, ++argv ) {
        if( !strcmp(argv[0], "--tmp-path") ) {
            tmp_path = argv[1];
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-c") || !strcmp(argv[0], "--ncols") ) {
            ncols = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-l") || !strcmp(argv[0], "--nlabels") ) {
            nlabels = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-r") || !strcmp(argv[0], "--nrows") ) {
            nrows = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-n") || !strcmp(argv[0], "--nsteps") ) {
            nsteps = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-t") || !strcmp(argv[0], "--ntrials") ) {
            ntrials = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--pa") ) {
            pa = strtod(argv[1], 0);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--po") ) {
            po = strtod(argv[1], 0);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--kappa") ) {
            epsilon_for_kappa = strtod(argv[1], 0);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--map-epsilon") ) {
            map_epsilon = strtod(argv[1], 0);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--map-threshold") ) {
            map_threshold = strtod(argv[1], 0);
            --argc;
            ++argv;
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
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "--read-execution") ) {
            read_execution = true;
            filename_execution = string(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--save-execution-and-exit") ) {
            save_execution_and_exit = true;
            filename_execution = string(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--task") ) {
            string task = argv[1];
            if( task == "color" ) {
                slam_type = cellmap_t::COLOR;
            } else if( task == "mine-mapping-peaked" ) {
                slam_type = cellmap_t::MINE_MAPPING_PEAKED;
            } else if( task == "mine-mapping-non-peaked" ) {
                slam_type = cellmap_t::MINE_MAPPING_NON_PEAKED;
            } else if( task == "1d" ) {
                slam_type = cellmap_t::CORRIDOR;
                nrows = 1;
            } else {
                cerr << "error: undefined task '" << task << "'. Valid values are 'color', 'mine-mapping-peaked', 'mine-mapping-non-peaked', and '1d'." << endl;
                exit(-1);
            }
            --argc;
            ++argv;
        } else if( !strncmp(argv[0], "--tracker", 10) ) {
            string tracker = argv[1];
            Utils::tokenize(tracker, tracker_strings);
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "--plot") || !strcmp(argv[0], "--generate-plot-R") ) {
            R_plot = true;
        } else if( !strcmp(argv[0], "-v") || !strcmp(argv[0], "--verbose") ) {
            verbose = true;
        } else if( !strcmp(argv[0], "-?") || !strcmp(argv[0], "--help") ) {
            usage(cerr);
            exit(-1);
        } else {
            cerr << "error: unexpected argument: " << argv[0] << endl;
            exit(-1);
        }
    }

    // check proper dimension for 1d-slam
    if( (slam_type == cellmap_t::CORRIDOR) && (nrows != 1) ) {
        cerr << "error: number of rows for 1d-slam must be equal to 1" << endl;
        exit(-1);
    }

    // set seed
    Utils::set_seed(seed);
    cout << "# seed=" << seed << endl;

    // create cellmap
    cellmap_t cellmap(nrows, ncols, nlabels, slam_type, pa, po);

    // set static members
    coord_t::ncols_ = ncols;
    base_particle_t::base_ = &cellmap;
    Decomposable::decomposable_t<cellmap_t>::static_base_ = &cellmap; // CHECK: remove this stuff used in var_offset(). Hide it inside cellmap.h

    if( !save_execution_and_exit ) {
        //Inference::edbp_t::initialize();
        //Inference::inference_t::initialize_edbp(tmp_path);
        kappa_t::initialize(epsilon_for_kappa, 10);
        Decomposable::cache_t::initialize(nrows, ncols);
        if( (slam_type == cellmap_t::MINE_MAPPING_PEAKED) || (slam_type == cellmap_t::MINE_MAPPING_NON_PEAKED) ) {
            MineMapping::cache_t::initialize(nrows, ncols);
            MineMapping::kappa_arc_consistency_t::initialize(nrows, ncols);
#if 0 // V2
            MineMappingV2::cache_t::initialize(nrows, ncols);
            MineMappingV2::kappa_arc_consistency_t::initialize(nrows, ncols);
#endif
        }
        if( slam_type == cellmap_t::CORRIDOR ) {
            Corridor::cache_t::initialize(nrows, ncols);
            Corridor::kappa_arc_consistency_t::initialize(nrows, ncols);
        }
    }

    // tracking algorithms
    int nparticles = 10; // default value
    vector<tracking_t<cellmap_t>*> trackers;
    for( size_t i = 0; i < tracker_strings.size(); ++i ) {
        tracking_t<cellmap_t> *tracker = 0;

        const string &name = tracker_strings[i];
        string short_name;
        string parameter_str;
        multimap<string, string> parameters;
        Utils::split_request(name, short_name, parameter_str);
        Utils::tokenize(parameter_str, parameters);

        multimap<string, string>::const_iterator it = parameters.find("nparticles");
        if( it == parameters.end() )
            parameters.insert(make_pair(string("nparticles"), to_string(nparticles)));
        else
            nparticles = strtol(it->second.c_str(), 0, 0);

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

        if( short_name == "decomposable" ) {
            tracker = new Decomposable::decomposable_t<cellmap_t>(name, cellmap, parameters);
        } else if( short_name == "sis" ) {
            tracker = new SIS_t<sis_slam_particle_t, cellmap_t>(name, cellmap, parameters);
        } else if( short_name == "mm-sir" ) {
            tracker = new motion_model_SIR_t<motion_model_sir_slam_particle_t, cellmap_t>(name, cellmap, parameters);
        } else if( short_name == "opt-sir" ) {
            cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t> *cdf =
              new cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t>(cellmap);
            tracker = new optimal_SIR_t<optimal_sir_slam_particle_t, cellmap_t, cdf_for_optimal_sir_t<optimal_sir_slam_particle_t, cellmap_t> >(name, cellmap, parameters, *cdf);
        } else if( short_name == "mm-rbpf" ) {
            if( slam_type == cellmap_t::COLOR ) {
                tracker = new RBPF_t<motion_model_rbpf_slam_particle_t, cellmap_t>(name, cellmap, parameters);
            } else if( (slam_type == cellmap_t::MINE_MAPPING_PEAKED) || (slam_type == cellmap_t::MINE_MAPPING_NON_PEAKED) ) {
                tracker = new RBPF_t<MineMapping::motion_model_rbpf_particle_t, cellmap_t>(name, cellmap, parameters);
            } else {
                assert(slam_type == cellmap_t::CORRIDOR);
                tracker = new RBPF_t<Corridor::motion_model_rbpf_particle_t, cellmap_t>(name, cellmap, parameters);
            }
        } else if( short_name == "opt-rbpf" ) {
            if( slam_type == cellmap_t::COLOR ) {
                tracker = new RBPF_t<optimal_rbpf_slam_particle_t, cellmap_t>(name, cellmap, parameters);
            } else if( (slam_type == cellmap_t::MINE_MAPPING_PEAKED) || (slam_type == cellmap_t::MINE_MAPPING_NON_PEAKED) ) {
                tracker = new RBPF_t<MineMapping::optimal_rbpf_particle_t, cellmap_t>(name, cellmap, parameters);
            } else {
                assert(slam_type == cellmap_t::CORRIDOR);
                tracker = new RBPF_t<Corridor::optimal_rbpf_particle_t, cellmap_t>(name, cellmap, parameters);
            }
#if 0 // V2
        } else if( short_name == "mm-rbpf2" ) {
            assert((slam_type == cellmap_t::MINE_MAPPING_PEAKED) || (slam_type == cellmap_t::MINE_MAPPING_NON_PEAKED));
            tracker = new RBPF_t<MineMappingV2::motion_model_rbpf_particle_t, cellmap_t>(name, cellmap, parameters);
        } else if( short_name == "opt-rbpf2" ) {
            assert((slam_type == cellmap_t::MINE_MAPPING_PEAKED) || (slam_type == cellmap_t::MINE_MAPPING_NON_PEAKED));
            tracker = new RBPF_t<MineMappingV2::optimal_rbpf_particle_t, cellmap_t>(name, cellmap, parameters);
#endif
        } else {
            cerr << "warning: unrecognized tracking algorithm '" << name << "'" << endl;
        }
        if( tracker != 0 ) trackers.push_back(tracker);
    }

    // check that there is something to do
    if( !save_execution_and_exit && trackers.empty() ) {
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

    // input/output streams for saving/reading executions
    if( read_execution ) {
        ifs_execution = new ifstream(filename_execution);
        *ifs_execution >> ntrials;
    } else if( save_execution_and_exit ) {
        ofs_execution = new ofstream(filename_execution);
        *ofs_execution << ntrials;
    }

    // run for the specified number of trials and collect statistics
    cout << fixed;
    int total_execution_length = 0;
    cellmap_t::execution_t fixed_execution;
    vector<Utils::stat_t> stats_unknown(trackers.size());
    vector<Utils::stat_t> stats_error(trackers.size());
    for( int trial = 0; trial < ntrials; ++trial ) {
        cout << "# trial = " << 1 + trial << " / " << ntrials << endl;
        if( save_execution_and_exit ) *ofs_execution << " ";

        // reading/saving executions
        if( read_execution ) {
            if( !read_labels_and_execution(*ifs_execution, cellmap, fixed_execution) )
                break;
        } else {
            // set labels and fixed execution (if appropriate)
            set_labels_and_execution(cellmap, ptype, num_covering_loops, fixed_execution);
            if( save_execution_and_exit ) {
                write_labels_and_execution(*ofs_execution, cellmap, fixed_execution);
                total_execution_length += fixed_execution.size();
                continue;
            }
        }

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
        total_execution_length += output_execution.size();

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
                                string("dummy"),
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
                     << ":" << setprecision(3) << map_values.back().first;
            } else {
                cout << "{";
                for( size_t k = 0; k < map_values.size(); ++k ) {
                    cout << coord_t(map_values[k].second) << ":" << map_values[k].second << ":" << setprecision(3) << map_values[k].first;
                    if( k < map_values.size() - 1 ) cout << ",";
                }
                cout << "}";
            }

            cout << ", #steps=" << output_execution.size()
                 << ", #unknowns=" << unknowns
                 << ", #errors=" << errors
                 << ", elapsed-time=" << setprecision(5) << tracker.elapsed_time()
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
    if( !save_execution_and_exit ) {
        for( size_t i = 0; i < trackers.size(); ++i ) {
            const tracking_t<cellmap_t> &tracker = *trackers[i];
            cout << "# stats(" << setw(size_longest_name) << tracker.name_ << "):"
                 << " trials=" << int(stats_unknown[i].n())
                 << ", total-elapsed-time=" << setprecision(5) << tracker.total_elapsed_time()
                 << ", avg-unknowns-per-trial=" << setprecision(3) << stats_unknown[i].mean() << " (" << stats_unknown[i].confidence(.95) << ")"
                 << ", avg-errors-per-trial=" << setprecision(3) << stats_error[i].mean() << " (" << stats_error[i].confidence(.95) << ")"
                 << ", avg-elapsed-time-per-trial=" << setprecision(5) << tracker.total_elapsed_time() / ntrials
                 << ", avg-elapsed-time-per-step=" << setprecision(5) << tracker.total_elapsed_time() / tracker.total_num_steps()
                 << endl;
        }
    }
    cout << "# avg execution length = " << float(total_execution_length) / float(ntrials) << endl;

    // input/output streams for saving/reading executions
    if( ifs_execution != 0 ) ifs_execution->close();
    if( ofs_execution != 0 ) {
        *ofs_execution << endl;
        ofs_execution->close();
    }
    delete ifs_execution;
    delete ofs_execution;

    // stop timer
    float elapsed_time = Utils::read_time_in_seconds() - start_time;
    cout << "# total time = " << setprecision(5) << elapsed_time << endl;

    Inference::inference_t::finalize_edbp();
    finalize();
    return 0;
}

