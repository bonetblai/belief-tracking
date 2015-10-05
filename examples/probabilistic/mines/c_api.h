void agent_initialize(int rows, int cols, int nmines);
void agent_finalize();
int agent_get_action(int *is_guess);
int agent_is_flag_action(int action);
int agent_get_cell(int action);
void agent_increase_nguesses(int n);
int agent_get_nguesses();
void agent_increase_ninferences(int n);
void agent_update_state(int flag, int cell, int obs);
void agent_declare_win(bool output);
void agent_declare_lose(bool output);

