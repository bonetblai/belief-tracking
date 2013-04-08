#include <iostream>
#include "DiagonalWumpusAgentProxy.h"
#include <diagonal_wumpus/wumpus_api.h>

using namespace std;

static Wumpus::Diagonal::abstract_api_t *wumpus_agent = 0;

JNIEXPORT void JNICALL Java_DiagonalWumpusAgentProxy_init_1cpp_1side
  (JNIEnv *env, jobject, jint dim) {

#if 0
    cout << "Proxy:" << " dim=" << dim << endl;
#endif

    int num_expansions = 400;
    int mdp_horizon = 4 * dim;
    int expansions_per_iteration = num_expansions / 10;

    wumpus_agent = new Wumpus::Diagonal::wumpus_api_t(dim);
    //wumpus_agent->select_policy("greedy_wrt_sduv-heuristic", "direct");
    wumpus_agent->set_policy_parameters(num_expansions, mdp_horizon, 0.5, expansions_per_iteration);
    wumpus_agent->select_policy("shortest_distance_to_unvisited_cell_heuristic", "aot/heuristic");
    //wumpus_agent->select_policy("greedy_wrt_sduv-heuristic", "aot");
}

JNIEXPORT void JNICALL Java_DiagonalWumpusAgentProxy_set_1policy_1parameters
  (JNIEnv *env, jobject, jint num_expansions, jint mdp_horizon) {
    int expansions_per_iteration = num_expansions / 10;
    wumpus_agent->set_policy_parameters(num_expansions, mdp_horizon, 0.5, expansions_per_iteration);
}

JNIEXPORT void JNICALL Java_DiagonalWumpusAgentProxy_prepare_1new_1trial
  (JNIEnv *env, jobject) {
    //cout << "entry: prepare_new_trial" << endl;
    wumpus_agent->prepare_new_trial(North);
    //cout << "exit: prepare_new_trial" << endl;
}

JNIEXPORT void JNICALL Java_DiagonalWumpusAgentProxy_update
  (JNIEnv *env, jobject, jint obs) {
    //cout << "entry: update" << endl;
    wumpus_agent->update(obs);
    //cout << "exit: update" << endl;
}

JNIEXPORT void JNICALL Java_DiagonalWumpusAgentProxy_apply
  (JNIEnv *, jobject, jint action) {
    wumpus_agent->apply(action);
}

JNIEXPORT jint JNICALL Java_DiagonalWumpusAgentProxy_select_1action
  (JNIEnv *env, jobject) {
    //cout << "entry: select_action" << endl;
    return wumpus_agent->select_action();
}

