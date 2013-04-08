
#include <iostream>
//#include "grid_belief.h"
//#include "defs.h"
#include "wumpus_api.h"

using namespace std;

int main(int argc, const char **argv) {
    int dim = atoi(*++argv);
    cout << "dim=" << dim << endl;

    Wumpus::Diagonal::wumpus_api_t api(dim);
    api.set_policy_parameters(5000, 2 * dim, 0.5, 10);
    api.select_policy("shortest_distance_to_unvisited_cell_heuristic", "aot/heuristic");
    api.prepare_new_trial(0);

    int action = api.select_action();
    cout << "action=" << action << endl;

    return 0;
}

