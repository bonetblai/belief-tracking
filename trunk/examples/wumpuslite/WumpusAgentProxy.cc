#include <iostream>
#include "WumpusAgentProxy.h"
#include <wumpus/wumpus_api.h>

using namespace std;

static Wumpus::abstract_api_t *wumpus_agent = 0;

//static const char *base = "shortest_distance_to_unvisited_cell_heuristic";
//static const char *base = "greedy_wrt_sduv-heuristic";
//static const char *base = "random";

//static const char *policy = "direct";
//static const char *policy = "aot/heuristic,random-ties";
//static const char *policy = "aot/random-ties";
//static const char *policy = "uct/random-ties";

JNIEXPORT void JNICALL Java_WumpusAgentProxy_init_1cpp_1side
  (JNIEnv *env, jobject,
   jboolean moving, jint rows, jint cols, jint npits,
   jint nwumpus, jint narrows, jboolean nesw_movements) {

#if 0
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
          new Wumpus::moving_wumpus_api_t(rows, cols, npits, nwumpus, narrows, nesw_movements, moving);
    } else {
        wumpus_agent =
          new Wumpus::wumpus_api_t(rows, cols, npits, nwumpus, narrows, nesw_movements, moving);
    }

    if( wumpus_agent->is_moving() ) {
        wumpus_agent->set_policy_parameters(100, 50, .5, 5);
        wumpus_agent->select_policy("shortest_distance_to_unvisited_cell_heuristic", "aot/heuristic");
    } else {
        wumpus_agent->select_policy("greedy_wrt_sduv-heuristic", "direct");
    }
}

JNIEXPORT void JNICALL Java_WumpusAgentProxy_set_1policy_1parameters
  (JNIEnv *env, jobject, jint num_expansions, jint mdp_horizon) {
}

JNIEXPORT void JNICALL Java_WumpusAgentProxy_prepare_1new_1trial
  (JNIEnv *env, jobject, jboolean diagonal) {
    //cout << "entry: prepare_new_trial" << endl;
    wumpus_agent->prepare_new_trial(Wumpus::North, diagonal);
    //cout << "exit: prepare_new_trial" << endl;
}

JNIEXPORT void JNICALL Java_WumpusAgentProxy_update
  (JNIEnv *env, jobject, jint obs) {
    //cout << "entry: update" << endl;
    wumpus_agent->update(obs);
    //cout << "exit: update" << endl;
}

JNIEXPORT void JNICALL Java_WumpusAgentProxy_apply_1action_1and_1update
  (JNIEnv *env, jobject, jint action, jint obs) {
    //cout << "entry: apply_and_update" << endl;
    wumpus_agent->apply_action_and_update(action, obs);
    //wumpus_agent->print(std::cout);
    //cout << "is_world_explored=" << (wumpus_agent->is_world_explored() ? 1 : 0) << endl;
    //cout << "exit: apply_and_update" << endl;
}

JNIEXPORT jint JNICALL Java_WumpusAgentProxy_select_1action
  (JNIEnv *env, jobject) {
    //cout << "entry: select_action" << endl;
    return wumpus_agent->select_action();
}

JNIEXPORT jint JNICALL Java_WumpusAgentProxy_is_1world_1explored
  (JNIEnv *env, jobject) {
    return wumpus_agent->is_world_explored();
}

JNIEXPORT jint JNICALL Java_WumpusAgentProxy_is_1there_1a_1safe_1cell
  (JNIEnv *env, jobject) {
    return wumpus_agent->is_there_an_unvisited_safe_cell();
}

