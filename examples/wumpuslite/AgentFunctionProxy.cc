#include <iostream>
#include "AgentFunctionProxy.h"
#include <wumpus_api.h>

using namespace std;

abstract_api_t *wumpus_agent = 0;

JNIEXPORT void JNICALL Java_AgentFunctionProxy_init_1cpp_1side
  (JNIEnv *env, jobject,
   jboolean moving, jint rows, jint cols, jint npits,
   jint nwumpus, jint narrows, jboolean nesw_movements) {

#if 1
    cout << "Proxy:"
         << " moving=" << (moving ? 1 : 0)
         << ", rows=" << rows
         << ", cols=" << cols
         << ", npits=" << npits
         << ", nwumpus=" << nwumpus
         << ", narrows=" << narrows
         << ", nesw-movements=" << (nesw_movements ? 1 : 0)
         << endl;
#endif

    if( moving ) {
        wumpus_agent =
          new moving_wumpus_api_t(rows, cols, npits, nwumpus, narrows, nesw_movements);
    } else {
        wumpus_agent =
          new wumpus_api_t(rows, cols, npits, nwumpus, narrows, nesw_movements);
    }
    //wumpus_agent->set_policy_parameters(50, 50, 0.5, 10);
    //wumpus_agent->select_policy("shortest_distance_to_unvisited_cell_heuristic", "aot/heuristic,random-ties");
    //wumpus_agent->select_policy("random", "direct");
    wumpus_agent->select_policy("greedy_wrt_sduv-heuristic", "direct");
}

JNIEXPORT void JNICALL Java_AgentFunctionProxy_set_1policy_1parameters
  (JNIEnv *env, jobject, jint num_expansions, jint mdp_horizon) {
    //wumpus_agent->set_policy_parameters(num_expansions, mdp_horizon, 0.5, 10);
    //wumpus_agent->select_policy("shortest_distance_to_unvisited_cell_heuristic", "aot/heuristic,random-ties");
    //wumpus_agent->select_policy("random", "direct");
    //wumpus_agent->select_policy("greedy_wrt_sduv-heuristic", "direct");
}

JNIEXPORT void JNICALL Java_AgentFunctionProxy_prepare_1new_1trial
  (JNIEnv *env, jobject) {
    //cout << "entry: prepare_new_trial" << endl;
    wumpus_agent->prepare_new_trial(North);
    //cout << "exit: prepare_new_trial" << endl;
}

JNIEXPORT void JNICALL Java_AgentFunctionProxy_update
  (JNIEnv *env, jobject, jint obs) {
    //cout << "entry: update" << endl;
    wumpus_agent->update(obs);
    //cout << "exit: update" << endl;
}

JNIEXPORT void JNICALL Java_AgentFunctionProxy_apply_1action_1and_1update
  (JNIEnv *env, jobject, jint action, jint obs) {
    //cout << "entry: apply_and_update" << endl;
    wumpus_agent->apply_action_and_update(action, obs);
    //wumpus_agent->print(std::cout);
    //cout << "is_world_explored=" << (wumpus_agent->is_world_explored() ? 1 : 0) << endl;
    //cout << "exit: apply_and_update" << endl;
}

JNIEXPORT jint JNICALL Java_AgentFunctionProxy_select_1action
  (JNIEnv *env, jobject) {
    //cout << "entry: select_action" << endl;
    return wumpus_agent->select_action();
}

JNIEXPORT jint JNICALL Java_AgentFunctionProxy_is_1world_1explored
  (JNIEnv *env, jobject) {
    return wumpus_agent->is_world_explored();
}

JNIEXPORT jint JNICALL Java_AgentFunctionProxy_is_1there_1a_1safe_1cell
  (JNIEnv *env, jobject) {
    return wumpus_agent->is_there_an_unvisited_safe_cell();
}

