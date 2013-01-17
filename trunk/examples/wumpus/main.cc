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

#include <iostream>
#include <fstream>
#include <vector>

#include "wumpus.h"
#include "base_policy.h"

#define EXPERIMENT

#include "../evaluation.h"
#include "dispatcher.h"

using namespace std;

namespace Online {
  namespace Policy {
    namespace AOT {
      const Heuristic::heuristic_t<state_t> *global_heuristic = 0;
    };
    namespace AOT2 {
      const Heuristic::heuristic_t<state_t> *global_heuristic = 0;
    };
  };
};

void usage(ostream &os) {
    os << "usage: wumpus [-s <seed>] <nrows> <ncols> <npits> <nwumpus>"
       << endl;
}

int main(int argc, const char **argv) {
    unsigned bitmap = 0;
    int h = 0;
    bool formatted = false;
    float dead_end_value = 1e5;
    bool nesw_movements = false;

    int nrows = 0;
    int ncols = 0;
    int npits = 0;
    int nwumpus = 0;
    int narrows = 0;

    string base_name;
    string policy_type;
    Evaluation::parameters_t eval_pars;

    cout << fixed;
    Algorithm::parameters_t alg_pars;

    // parse arguments
    ++argv;
    --argc;
    while( argc > 1 ) {
        if( **argv != '-' ) break;
        switch( (*argv)[1] ) {
            case 'a':
                bitmap = strtoul(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            case 'b':
                alg_pars.rtdp.bound_ = strtol(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            case 'c':
                nesw_movements = true;
                ++argv;
                --argc;
                break;
            case 'd':
                dead_end_value = strtod(argv[1], 0);
                argv += 2;
                argc -= 2;
                break;
            case 'D':
                eval_pars.evaluation_depth_ = strtoul(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            case 'e':
                alg_pars.epsilon_ = strtod(argv[1], 0);
                argv += 2;
                argc -= 2;
                break;
            case 'f':
                formatted = true;
                ++argv;
                --argc;
                break;
            case 'g':
                alg_pars.rtdp.epsilon_greedy_ = strtod(argv[1], 0);
                argv += 2;
                argc -= 2;
                break;
            case 'h':
                h = strtol(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            case 's':
                alg_pars.seed_ = strtoul(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            case 't':
                eval_pars.evaluation_trials_ = strtoul(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            default:
                usage(cout);
                exit(-1);
        }
    }

    if( argc >= 7 ) {
        nrows = strtoul(*argv++, 0, 0);
        ncols = strtoul(*argv++, 0, 0);
        npits = strtoul(*argv++, 0, 0);
        nwumpus = strtoul(*argv++, 0, 0);
        narrows = strtoul(*argv++, 0, 0);
        argc -= 5;
        base_name = *argv++;
        policy_type = *argv++;
        argc -= 2;
        if( argc-- > 0 ) eval_pars.width_ = strtoul(*argv++, 0, 0);
        if( argc-- > 0 ) eval_pars.depth_ = strtoul(*argv++, 0, 0);
        if( argc-- > 0 ) eval_pars.par1_ = strtod(*argv++, 0);
        if( argc-- > 0 ) eval_pars.par2_ = strtod(*argv++, 0);
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    cout << "seed=" << alg_pars.seed_ << endl;
    Random::seeds(alg_pars.seed_);

    state_t::initialize(nrows, ncols);
    wumpus_belief_t::initialize(nrows, ncols, npits, nwumpus);
    problem_t problem(nrows, ncols, npits, nwumpus, narrows, dead_end_value);

    // create heuristic
    Heuristic::heuristic_t<state_t> *heuristic = 0;
    if( (h == 1) || (h = 11) ) {
        heuristic = new shortest_distance_to_unvisited_cell_t<state_t>(problem, nesw_movements);
        if( h == 11 ) {
            Onlie::Policy::AOT::global_heuristic = heuristic;
            Online::Policy::AOT2::global_heuristic = heuristic;
        }
    }

    // solve problem with algorithms
    vector<Dispatcher::result_t<state_t> > results;
    Dispatcher::solve(problem, heuristic, problem.init(), bitmap, alg_pars, results);

    m_state_t::initialize(nrows, ncols);
    m_wumpus_belief_t::initialize(nrows, ncols, npits, nwumpus);
    m_problem_t m_problem(nrows, ncols, npits, nwumpus, narrows, dead_end_value);
    Heuristic::heuristic_t<m_state_t> *m_heuristic = new shortest_distance_to_unvisited_cell_t<m_state_t>(m_problem, nesw_movements);
    vector<Dispatcher::result_t<m_state_t> > m_results;
    Dispatcher::solve(m_problem, m_heuristic, m_problem.init(), bitmap, alg_pars, m_results);

    // print results
    if( !results.empty() ) {
        if( formatted ) Dispatcher::print_result<state_t>(cout, 0);
        for( unsigned i = 0; i < results.size(); ++i ) {
            Dispatcher::print_result(cout, &results[i]);
        }
    }

    // evaluate policies
    vector<pair<const Policy::policy_t<state_t>*, string> > bases;

    // fill base policies
    const Problem::hash_t<state_t> *hash = results.empty() ? 0 : results[0].hash_;
    if( hash != 0 ) {
        Policy::hash_policy_t<state_t> optimal(*hash);
        bases.push_back(make_pair(optimal.clone(), "optimal"));
    }
    if( heuristic != 0 ) {
        Policy::greedy_t<state_t> greedy(problem, *heuristic);
        bases.push_back(make_pair(greedy.clone(), "greedy"));
    }
    Policy::random_t<state_t> random(problem);
    bases.push_back(make_pair(&random, "random"));
    //wumpus_base_policy_t<state_t> wumpus(problem, nrows, ncols, nesw_movements);
    //bases.push_back(make_pair(&wumpus, "wumpus_base"));

    // evaluate
    pair<const Policy::policy_t<state_t>*, std::string> policy = Evaluation::select_policy(base_name, policy_type, bases, eval_pars);
    if( policy.first != 0 ) {
        vector<float> values;
        values.reserve(eval_pars.evaluation_trials_);
        float start_time = Utils::read_time_in_seconds();
        float sum = 0;
        int ndead = 0, ngold = 0;
        cout << "#trials=" << eval_pars.evaluation_trials_ << ":";

        hidden_state_t hidden(nrows, ncols, npits, nwumpus, narrows);
        for( unsigned trial = 0; trial < eval_pars.evaluation_trials_; ++trial ) {
            cout << " " << trial << flush;

            // sample hidden state
            hidden.sample(0, North, npits, nwumpus);

            // set initial belief state
            state_t state(narrows);
            state.set_initial_configuration();

            // filter with initial obs
            int obs = hidden.get_obs();
            cout << "initial obs=" << obs_name(obs) << endl;
            state.update(-1, obs);

//cout << "initial: " << state;
//vector<pair<state_t, float> > outcomes;
//problem.next(state, 0, outcomes);
//cout << outcomes.size() << endl;
//for( int i = 0, isz = outcomes.size(); i < isz; ++i )
//    cout << outcomes[i].second << endl;
//exit(0);

            // do evaluation from start node
            size_t steps = 0;
            float cost = 0;
            while( (steps < eval_pars.evaluation_depth_) && !problem.terminal(state) ) {
                //cout << "state: " << state;
                assert(hidden.position() == state.position());
                assert(!state.dead());
        
                //cout << "selecting action: " << flush; 
                Problem::action_t action = (*policy.first)(state);
                assert(action != Problem::noop);
                assert(state.applicable(action));
                assert(hidden.applicable(action));

                int obs = hidden.apply_action_and_get_obs(action);
                cout << "pos=(" << (state.position() % ncols)
                     << "," << (state.position() / ncols)
                     << "," << heading_name(state.heading())
                     << "), action=" << action_name(action, nesw_movements)
                     << ", obs=" << obs_name(obs) << endl;

                if( hidden.dead() ) {
                    cost += 1e5;
                    break;
                } else {
                    state.apply_action_and_update(action, obs);
                    cost += problem.cost(state, action);
                    if( state.dead() ) assert(hidden.dead());
                }
                ++steps;
            }
            ndead += hidden.dead() ? 1 : 0;
            ngold += hidden.have_gold() ? 1 : 0;
            values.push_back(cost);
            sum += cost;
            if( hidden.dead() ) cout << "D";
            if( hidden.have_gold() ) cout << "G";
            cout << "(" << setprecision(1) << sum/(1+trial) << ")" << flush;
        }
        cout << endl;
        cout << "dead avg. = " << (float)ndead / (float)eval_pars.evaluation_trials_ << std::endl;
        cout << "gold avg. = " << (float)ngold / (float)eval_pars.evaluation_trials_ << std::endl;

        // compute avg
        float avg = 0;
        for( unsigned i = 0; i < eval_pars.evaluation_trials_; ++i ) {
            avg += values[i];
        }
        avg /= eval_pars.evaluation_trials_;

        // compute stdev
        float stdev = 0;
        for( unsigned i = 0; i < eval_pars.evaluation_trials_; ++i ) {
            stdev += (avg - values[i]) * (avg - values[i]);
        }
        stdev = sqrt(stdev) / (eval_pars.evaluation_trials_ - 1);

        cout << policy.second
             << "= " << setprecision(5) << avg
             << " " << stdev << setprecision(2)
             << " ( " << Utils::read_time_in_seconds() - start_time
             << " secs " << policy.first->decisions() << " decisions)" << std::endl;
        policy.first->print_stats(cout);
    } else {
        cout << "error: " << policy.second << endl;
    }

    // free resources
    delete policy.first;
    for( unsigned i = 0; i < results.size(); ++i ) {
        delete results[i].hash_;
    }
    delete heuristic;

    exit(0);
}

