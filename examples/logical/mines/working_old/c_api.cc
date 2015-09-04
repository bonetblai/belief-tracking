#include <iostream>
#include <sys/resource.h>
#include <sys/time.h>

extern "C" {
#include "c_api.h"
};
#include "agent.h"

inline float read_time_in_seconds() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (float)r_usage.ru_utime.tv_sec +
           (float)r_usage.ru_utime.tv_usec / (float)1000000;
}

using namespace std;

state_t *agent_state = 0;
base_policy_t *agent_policy = 0;

int ngames = 0, nwins = 0, nguesses = 0;
float start_time = 0;
float elapsed_time = 0;
bool playing = false;

void agent_initialize(int rows, int cols, int nmines) {
    delete agent_state;
    delete agent_policy;

#if 0
    cout << "agent: initialization: rows=" << rows
         << ", cols=" << cols
         << ", nmines=" << nmines
         << endl;
#endif

    mines_belief_t::initialize(rows, cols);
    agent_state = new state_t(rows, cols, nmines);
    agent_state->set_as_unknown();
    agent_policy = new base_policy_t;
    if( !playing ) {
        ++ngames;
        playing = true;
    }
    start_time = read_time_in_seconds();
}

int agent_get_action() {
    assert(agent_policy != NULL);
    pair<int, bool> p = (*agent_policy)(*agent_state);
    nguesses += p.second ? 1 : 0;
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
       << ", etime=" << elapsed_time
       << ", etime/game=" << elapsed_time / (float)ngames
       << endl;
}

void agent_declare_win() {
    playing = false;
    ++nwins;
    elapsed_time += read_time_in_seconds() - start_time;
    print_stats(std::cout);
}

void agent_declare_lose() {
    playing = false;
    elapsed_time += read_time_in_seconds() - start_time;
    print_stats(std::cout);
}

