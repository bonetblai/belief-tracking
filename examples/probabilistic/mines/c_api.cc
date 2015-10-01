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

state_t *g_agent_state = 0;
base_policy_t *g_agent_policy = 0;

int g_ngames = 0, g_nwins = 0, g_nguesses = 0, g_ndecisions = 0;
float g_start_time = 0;
float g_elapsed_time = 0;
bool g_playing = false;

void agent_initialize(int nrows, int ncols, int nmines) {
    delete g_agent_state;
    delete g_agent_policy;

#if 0
    cout << "agent: initialization: rows=" << rows
         << ", cols=" << cols
         << ", nmines=" << nmines
         << endl;
#endif

    state_t::initialize(nrows, ncols, nmines);
    g_agent_state = new state_t();
    g_agent_state->set_initial_configuration();
    g_agent_policy = new base_policy_t;
    if( !g_playing ) {
        ++g_ngames;
        g_playing = true;
    }
    g_start_time = read_time_in_seconds();
}

void agent_finalize() {
    delete g_agent_state;
    delete g_agent_policy;
    g_agent_state = 0;
    g_agent_policy = 0;
    state_t::finalize();
}

int agent_get_action(int *is_guess) {
    assert(g_agent_policy != NULL);
    ++g_ndecisions;
    pair<int, bool> p = (*g_agent_policy)(*g_agent_state);
    g_nguesses += p.second ? 1 : 0;
    if( is_guess != 0 ) *is_guess = p.second ? 1 : 0;
    return p.first;
}

int agent_is_flag_action(int action) {
    assert(g_agent_state != NULL);
    return action < g_agent_state->ncells() ? 1 : 0;
}

int agent_get_cell(int action) {
    assert(g_agent_state != NULL);
    bool flag = action < g_agent_state->ncells();
    return flag ? action : action - g_agent_state->ncells();
}

void agent_increase_nguesses(int n) {
    g_nguesses += n;
}

int agent_get_nguesses() {
    return g_nguesses;
}

void agent_update_state(int flag, int cell, int obs) {
    if( g_agent_state != 0 ) {
        g_agent_state->apply(flag == 1, cell);
        g_agent_state->update(flag == 1, cell, obs);
    }
}

void print_stats(ostream &os) {
    os << "stats: #games=" << g_ngames
       << ", #wins=" << g_nwins
       << ", %win=" << (float)g_nwins / (float)g_ngames
       << ", #guesses=" << g_nguesses
       << ", #decisions=" << g_ndecisions
       << ", etime=" << g_elapsed_time
       << ", etime/game=" << g_elapsed_time / (float)g_ngames
       << endl;
}

void agent_declare_win(bool output) {
    g_playing = false;
    ++g_nwins;
    g_elapsed_time += read_time_in_seconds() - g_start_time;
    if( output ) print_stats(std::cout);
}

void agent_declare_lose(bool output) {
    g_playing = false;
    g_elapsed_time += read_time_in_seconds() - g_start_time;
    if( output ) print_stats(std::cout);
}

