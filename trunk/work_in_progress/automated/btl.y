%name BTL_Parser
%define ERROR log_error
%define ERROR_BODY = 0
%define ERROR_VERBOSE 1
%define LEX next_token
%define LEX_BODY = 0
%define DEBUG 1

%define INHERIT : public BTL_Base
%define CONSTRUCTOR_PARAM StringTable& t
%define CONSTRUCTOR_INIT : BTL_Base(t), error_flag(false)
%define MEMBERS \
  public: \
    virtual ~BTL_Parser() { } \
    virtual std::ostream& syntax_errors() = 0; \
    bool error_flag; \
  private: \
    std::vector<ForallEffect*> forall_effects;

%header{
#include <stdlib.h>
#include <string.h>
#include <list>
#include <sstream>
#include "base.h"
%}

%union {
    StringTable::Cell                *sym;
    BTL_Base::Atom                   *atom;
    BTL_Base::symbol_vec             *param;
    BTL_Base::variable_vec           *vparam;
    const BTL_Base::Condition        *condition;
    const BTL_Base::Effect           *effect;
    const BTL_Base::Invariant        *invariant;
    const BTL_Base::Clause           *clause;
    const BTL_Base::Oneof            *oneof;
    const BTL_Base::init_element_vec *ilist;
    int                              ival;
}

%token             TK_NEW_SYMBOL
                   TK_INPUT_PARAMETER_SYMBOL TK_PARAMETER_SYMBOL
                   TK_NEW_VAR_SYMBOL TK_VAR_SYMBOL TK_DEF_VAR_SYMBOL
                   TK_ACTION_SYMBOL
                   TK_INTEGER

%token             KW_BEGIN_INPUT_PARAMETERS KW_END_INPUT_PARAMETERS
                   KW_BEGIN_VARIABLES KW_END_VARIABLES
                   KW_BEGIN_DEFINED_VARIABLES KW_END_DEFINED_VARIABLES
                   KW_BEGIN_ACTIONS KW_END_ACTIONS
                   KW_BEGIN_AXIOMS KW_END_AXIOMS
                   KW_BEGIN_OBSERVABLES KW_END_OBSERVABLES
                   KW_BEGIN_INITIAL_BELIEF KW_END_INITIAL_BELIEF

%token             KW_BOOLEAN KW_INTEGER KW_PAIR KW_SUCH KW_THAT KW_IS
                   KW_FOR KW_WITH KW_PRECONDITION KW_IF KW_THEN
                   KW_AND KW_OR KW_NOT KW_SOME KW_TRUE KW_FALSE KW_MOD
                   KW_UNKNOWN KW_DUMMY
                   PLUS MINUS EQ NOT_EQ ASSIGN
                   COLON SEMICOLON LEFTSQPAR RIGHTSQPAR LEFTPAR RIGHTPAR

%left              PLUS MINUS KW_OR
%left              KW_AND

/*
%type <sym>        action_symbol any_symbol sensor_symbol axiom_symbol
%type <param>      atom_argument_list
%type <vparam>     typed_param_list typed_param_sym_list

%type <atom>       positive_literal negative_literal literal
%type <condition>  condition single_condition condition_list
%type <condition>  goal_list single_goal
%type <invariant>  invariant at_least_one_invariant at_most_one_invariant exactly_one_invariant
%type <clause>     clause
%type <oneof>      oneof
%type <effect>     atomic_effect positive_atomic_effect positive_atomic_effect_list
%type <effect>     action_effect action_effect_list single_action_effect conditional_effect forall_effect
%type <effect>     atomic_effect_kw_list atomic_effect_list
%type <ilist>      init_elements
*/


%start btl_declarations

%%

btl_declarations:
      btl_input_parameters btl_variables btl_defined_variables
      btl_actions btl_axioms btl_observables btl_initial_situation
    ;

btl_input_parameters:
      KW_BEGIN_INPUT_PARAMETERS input_parameter_list KW_END_INPUT_PARAMETERS
    | /* empty */
    ;

input_parameter_list:
      input_parameter_list input_parameter_def
    | input_parameter_def
    ;

input_parameter_def:
      new_input_parameter_symbol COLON primitive_type SEMICOLON
    ;

primitive_type:
      KW_INTEGER
    | KW_BOOLEAN
    ;

btl_variables:
      KW_BEGIN_VARIABLES variable_list KW_END_VARIABLES
    | /* empty */
    ;

variable_list:
      variable_list variable_def
    | variable_def
    ;

variable_def:
      new_variable_symbol_with_parameters SEMICOLON
    ;

new_variable_symbol_with_parameters:
      new_variable_symbol COLON fully_qualified_type parameters_description
    ;

fully_qualified_type:
      KW_BOOLEAN
    | KW_INTEGER LEFTSQPAR bounded_integer RIGHTSQPAR
    ;

bounded_integer:
      TK_INTEGER
    | input_parameter_symbol
    ;

parameters_description:
      KW_FOR parameter_description_list
    | /* empty */
    ;

parameter_description_list:
      parameter_description_list KW_AND single_parameter_description
    | single_parameter_description
    ;

single_parameter_description:
      TK_PARAMETER_SYMBOL KW_IS fully_qualified_type
    ;

btl_defined_variables:
      KW_BEGIN_DEFINED_VARIABLES defined_variable_list KW_END_DEFINED_VARIABLES
    | /* empty */
    ;

defined_variable_list:
      defined_variable_list defined_variable_def
    | defined_variable_def
    ;

defined_variable_def:
      new_defined_variable_symbol COLON fully_qualified_type SEMICOLON
    ;

btl_actions:
      KW_BEGIN_ACTIONS action_list KW_END_ACTIONS
    | /* empty */
    ;

action_list:
      action_list action_def
    | action_def
    ;

action_def:
      action_name_and_precondition action_effects
    ;

action_name_and_precondition:
      action_symbol_with_parameters action_precondition SEMICOLON
    ;

action_symbol_with_parameters:
      new_action_symbol parameters_description
    ;

action_precondition:
      KW_WITH KW_PRECONDITION condition
    | /* empty */
    ;

action_effects:
      action_effects named_conditional_effect
    | /* empty */
    ;

named_conditional_effect:
      action_symbol COLON conditional_effect SEMICOLON
    ;

conditional_effect:
      KW_IF condition KW_THEN effect
    ;

condition:
      constant_condition
    | conjunction_condition
    ;

constant_condition:
      KW_TRUE
    | KW_FALSE
    ;

conjunction_condition:
      conjunction_condition KW_AND literal_condition
    | literal_condition
    ;

literal_condition:
      atomic_condition
    | KW_NOT atomic_condition
    ;

atomic_condition:
      TK_VAR_SYMBOL
    | TK_VAR_SYMBOL EQ expr
    | TK_VAR_SYMBOL NOT_EQ expr
    ;

effect:
      effect atomic_effect
    | /* empty */
    ;

atomic_effect:
      TK_VAR_SYMBOL ASSIGN expr
    ;

btl_axioms:
      KW_BEGIN_AXIOMS axiom_list KW_END_AXIOMS
    | /* empty */
    ;

axiom_list:
      axiom_list axiom_def
    | axiom_def
    ;

axiom_def:
      variable_symbol ASSIGN expr
    ;

btl_observables:
      KW_BEGIN_OBSERVABLES observable_list KW_END_OBSERVABLES
    | /* empty */
    ;

observable_list:
      observable_list observable_def
    | observable_def
    ;

observable_def:
      variable_symbol SEMICOLON
    | defined_variable_symbol SEMICOLON
    ;

btl_initial_situation:
      KW_BEGIN_INITIAL_BELIEF init_list KW_END_INITIAL_BELIEF
    | /* empty */
    ;

init_list:
      init_list init_def
    | init_def
    ;

init_def:
      variable_symbol EQ expr SEMICOLON
    | KW_UNKNOWN variable_symbol SEMICOLON
    ;

// expresions

expr:
      expr PLUS term
    | expr MINUS term
    | expr KW_OR term
    | term
    ;

term:
      term KW_AND factor
    | factor
    ;

factor:
      constant_expr
    | variable_symbol
    | KW_NOT expr
    | LEFTPAR expr RIGHTPAR
    | LEFTPAR expr RIGHTPAR KW_MOD TK_INTEGER
    ;

constant_expr:
      KW_FALSE
    | KW_TRUE
    | TK_INTEGER
    ;

// symbols

new_input_parameter_symbol:
      TK_NEW_SYMBOL { }
    ;

input_parameter_symbol:
      TK_INPUT_PARAMETER_SYMBOL
    ;

new_variable_symbol:
      TK_NEW_VAR_SYMBOL { }
    ;

variable_symbol:
      TK_VAR_SYMBOL
    ;

new_defined_variable_symbol:
      TK_NEW_VAR_SYMBOL { }
    ;

defined_variable_symbol:
      TK_DEF_VAR_SYMBOL
    ;

new_action_symbol:
      TK_NEW_SYMBOL { }
    ;

action_symbol:
      TK_ACTION_SYMBOL
    ;

%%

