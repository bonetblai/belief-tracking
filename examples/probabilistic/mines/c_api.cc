#include <iostream>
#include <sys/resource.h>
#include <sys/time.h>

extern "C" {
#include "c_api.h"
};
#include "mines_agent.h"

inline float read_time_in_seconds() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    float time = (float)r_usage.ru_utime.tv_sec + (float)r_usage.ru_utime.tv_usec / (float)1000000;
    getrusage(RUSAGE_CHILDREN, &r_usage);
    time += (float)r_usage.ru_utime.tv_sec + (float)r_usage.ru_utime.tv_usec / (float)1000000;
    return time;
}

using namespace std;

state_t *agent_state = 0;
base_policy_t *agent_policy = 0;

int ngames = 0, nwins = 0, nguesses = 0, ndecisions = 0;
float start_time = 0;
float elapsed_time = 0;
bool playing = false;

void agent_initialize(int nrows, int ncols, int nmines) {
    delete agent_state;
    delete agent_policy;

#if 0
    cout << "agent: initialization: rows=" << rows
         << ", cols=" << cols
         << ", nmines=" << nmines
         << endl;
#endif

    state_t::initialize(nrows, ncols, nmines);
    agent_state = new state_t();
    agent_state->set_initial_configuration();
    agent_policy = new base_policy_t;
    if( !playing ) {
        ++ngames;
        playing = true;
    }
    start_time = read_time_in_seconds();
}

void agent_finalize() {
    delete agent_state;
    delete agent_policy;
    agent_state = 0;
    agent_policy = 0;
    state_t::finalize();
}

int agent_get_action(int *is_guess) {
    assert(agent_policy != NULL);
    ++ndecisions;
    pair<int, bool> p = (*agent_policy)(*agent_state);
    nguesses += p.second ? 1 : 0;
    if( is_guess != 0 ) *is_guess = p.second ? 1 : 0;
    return p.first;
}

int agent_is_flag_action(int action) {
    assert(agent_state != NULL);
    return action < agent_state->ncells() ? 1 : 0;
}

int agent_get_cell(int action) {
    assert(agent_state != NULL);
    bool flag = action < agent_state->ncells();
    return flag ? action : action - agent_state->ncells();
}

int agent_get_nguesses() {
    return nguesses;
}

void agent_update_state(int flag, int cell, int obs) {
    if( agent_state != 0 ) {
        agent_state->apply(flag == 1, cell);
        agent_state->update(flag == 1, cell, obs);
    }
}

void print_stats(ostream &os) {
    os << "stats: #games=" << ngames
       << ", #wins=" << nwins
       << ", %win=" << (float)nwins / (float)ngames
       << ", #guesses=" << nguesses
       << ", #decisions=" << ndecisions
       << ", etime=" << elapsed_time
       << ", etime/game=" << elapsed_time / (float)ngames
       << endl;
}

void agent_declare_win(bool output) {
    playing = false;
    ++nwins;
    elapsed_time += read_time_in_seconds() - start_time;
    if( output ) print_stats(std::cout);
}

void agent_declare_lose(bool output) {
    playing = false;
    elapsed_time += read_time_in_seconds() - start_time;
    if( output ) print_stats(std::cout);
}

