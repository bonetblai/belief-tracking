#ifndef YY_BTL_Parser_h_included
#define YY_BTL_Parser_h_included

#line 1 "/home/bonet/tools/lib/bison.h"
/* before anything */
#ifdef c_plusplus
#ifndef __cplusplus
#define __cplusplus
#endif
#endif
#ifdef __cplusplus
#ifndef YY_USE_CLASS
#define YY_USE_CLASS
#endif
#else
#endif
#include <stdio.h>

/* #line 14 "/home/bonet/tools/lib/bison.h" */
#line 21 "btl.tab.h"
#define YY_BTL_Parser_ERROR  log_error
#define YY_BTL_Parser_ERROR_BODY  = 0
#define YY_BTL_Parser_ERROR_VERBOSE  1
#define YY_BTL_Parser_LEX  next_token
#define YY_BTL_Parser_LEX_BODY  = 0
#define YY_BTL_Parser_DEBUG  1
#define YY_BTL_Parser_INHERIT  : public BTL_Base
#define YY_BTL_Parser_CONSTRUCTOR_PARAM  StringTable& t
#define YY_BTL_Parser_CONSTRUCTOR_INIT  : BTL_Base(t), error_flag(false)
#define YY_BTL_Parser_MEMBERS  \
  public: \
    virtual ~BTL_Parser() { } \
    virtual std::ostream& syntax_errors() = 0; \
    bool error_flag; \
  private: \
    std::vector<ForallEffect*> forall_effects;
#line 20 "btl.y"

#include <stdlib.h>
#include <string.h>
#include <list>
#include <sstream>
#include "base.h"

#line 28 "btl.y"
typedef union {
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
} yy_BTL_Parser_stype;
#define YY_BTL_Parser_STYPE yy_BTL_Parser_stype

#line 14 "/home/bonet/tools/lib/bison.h"
 /* %{ and %header{ and %union, during decl */
#ifndef YY_BTL_Parser_COMPATIBILITY
#ifndef YY_USE_CLASS
#define  YY_BTL_Parser_COMPATIBILITY 1
#else
#define  YY_BTL_Parser_COMPATIBILITY 0
#endif
#endif

#if YY_BTL_Parser_COMPATIBILITY != 0
/* backward compatibility */
#ifdef YYLTYPE
#ifndef YY_BTL_Parser_LTYPE
#define YY_BTL_Parser_LTYPE YYLTYPE
/* WARNING obsolete !!! user defined YYLTYPE not reported into generated header */
/* use %define LTYPE */
#endif
#endif
#ifdef YYSTYPE
#ifndef YY_BTL_Parser_STYPE 
#define YY_BTL_Parser_STYPE YYSTYPE
/* WARNING obsolete !!! user defined YYSTYPE not reported into generated header */
/* use %define STYPE */
#endif
#endif
#ifdef YYDEBUG
#ifndef YY_BTL_Parser_DEBUG
#define  YY_BTL_Parser_DEBUG YYDEBUG
/* WARNING obsolete !!! user defined YYDEBUG not reported into generated header */
/* use %define DEBUG */
#endif
#endif
#ifdef YY_BTL_Parser_STYPE
#ifndef yystype
#define yystype YY_BTL_Parser_STYPE
#endif
#endif
/* use goto to be compatible */
#ifndef YY_BTL_Parser_USE_GOTO
#define YY_BTL_Parser_USE_GOTO 1
#endif
#endif

/* use no goto to be clean in C++ */
#ifndef YY_BTL_Parser_USE_GOTO
#define YY_BTL_Parser_USE_GOTO 0
#endif

#ifndef YY_BTL_Parser_PURE

/* #line 63 "/home/bonet/tools/lib/bison.h" */
#line 114 "btl.tab.h"

#line 63 "/home/bonet/tools/lib/bison.h"
/* YY_BTL_Parser_PURE */
#endif

/* #line 65 "/home/bonet/tools/lib/bison.h" */
#line 121 "btl.tab.h"

#line 65 "/home/bonet/tools/lib/bison.h"
/* prefix */
#ifndef YY_BTL_Parser_DEBUG

/* #line 67 "/home/bonet/tools/lib/bison.h" */
#line 128 "btl.tab.h"

#line 67 "/home/bonet/tools/lib/bison.h"
/* YY_BTL_Parser_DEBUG */
#endif
#ifndef YY_BTL_Parser_LSP_NEEDED

/* #line 70 "/home/bonet/tools/lib/bison.h" */
#line 136 "btl.tab.h"

#line 70 "/home/bonet/tools/lib/bison.h"
 /* YY_BTL_Parser_LSP_NEEDED*/
#endif
/* DEFAULT LTYPE*/
#ifdef YY_BTL_Parser_LSP_NEEDED
#ifndef YY_BTL_Parser_LTYPE
typedef
  struct yyltype
    {
      int timestamp;
      int first_line;
      int first_column;
      int last_line;
      int last_column;
      char *text;
   }
  yyltype;

#define YY_BTL_Parser_LTYPE yyltype
#endif
#endif
/* DEFAULT STYPE*/
#ifndef YY_BTL_Parser_STYPE
#define YY_BTL_Parser_STYPE int
#endif
/* DEFAULT MISCELANEOUS */
#ifndef YY_BTL_Parser_PARSE
#define YY_BTL_Parser_PARSE yyparse
#endif
#ifndef YY_BTL_Parser_LEX
#define YY_BTL_Parser_LEX yylex
#endif
#ifndef YY_BTL_Parser_LVAL
#define YY_BTL_Parser_LVAL yylval
#endif
#ifndef YY_BTL_Parser_LLOC
#define YY_BTL_Parser_LLOC yylloc
#endif
#ifndef YY_BTL_Parser_CHAR
#define YY_BTL_Parser_CHAR yychar
#endif
#ifndef YY_BTL_Parser_NERRS
#define YY_BTL_Parser_NERRS yynerrs
#endif
#ifndef YY_BTL_Parser_DEBUG_FLAG
#define YY_BTL_Parser_DEBUG_FLAG yydebug
#endif
#ifndef YY_BTL_Parser_ERROR
#define YY_BTL_Parser_ERROR yyerror
#endif

#ifndef YY_BTL_Parser_PARSE_PARAM
#ifndef __STDC__
#ifndef __cplusplus
#ifndef YY_USE_CLASS
#define YY_BTL_Parser_PARSE_PARAM
#ifndef YY_BTL_Parser_PARSE_PARAM_DEF
#define YY_BTL_Parser_PARSE_PARAM_DEF
#endif
#endif
#endif
#endif
#ifndef YY_BTL_Parser_PARSE_PARAM
#define YY_BTL_Parser_PARSE_PARAM void
#endif
#endif

/* TOKEN C */
#ifndef YY_USE_CLASS

#ifndef YY_BTL_Parser_PURE
extern YY_BTL_Parser_STYPE YY_BTL_Parser_LVAL;
#endif


/* #line 143 "/home/bonet/tools/lib/bison.h" */
#line 214 "btl.tab.h"
#define	TK_NEW_SYMBOL	258
#define	TK_INPUT_PARAMETER_SYMBOL	259
#define	TK_PARAMETER_SYMBOL	260
#define	TK_NEW_VAR_SYMBOL	261
#define	TK_VAR_SYMBOL	262
#define	TK_DEF_VAR_SYMBOL	263
#define	TK_ACTION_SYMBOL	264
#define	TK_INTEGER	265
#define	KW_BEGIN_INPUT_PARAMETERS	266
#define	KW_END_INPUT_PARAMETERS	267
#define	KW_BEGIN_VARIABLES	268
#define	KW_END_VARIABLES	269
#define	KW_BEGIN_DEFINED_VARIABLES	270
#define	KW_END_DEFINED_VARIABLES	271
#define	KW_BEGIN_ACTIONS	272
#define	KW_END_ACTIONS	273
#define	KW_BEGIN_AXIOMS	274
#define	KW_END_AXIOMS	275
#define	KW_BEGIN_OBSERVABLES	276
#define	KW_END_OBSERVABLES	277
#define	KW_BEGIN_INITIAL_BELIEF	278
#define	KW_END_INITIAL_BELIEF	279
#define	KW_BOOLEAN	280
#define	KW_INTEGER	281
#define	KW_PAIR	282
#define	KW_SUCH	283
#define	KW_THAT	284
#define	KW_IS	285
#define	KW_FOR	286
#define	KW_WITH	287
#define	KW_PRECONDITION	288
#define	KW_IF	289
#define	KW_THEN	290
#define	KW_AND	291
#define	KW_OR	292
#define	KW_NOT	293
#define	KW_SOME	294
#define	KW_TRUE	295
#define	KW_FALSE	296
#define	KW_PLUS	297
#define	KW_MINUS	298
#define	KW_MOD	299
#define	KW_EQ	300
#define	KW_NEQ	301
#define	KW_ASSIGN	302
#define	KW_COLON	303
#define	KW_SEMICOLON	304
#define	KW_LEFTSQPAR	305
#define	KW_RIGHTSQPAR	306
#define	KW_LEFTPAR	307
#define	KW_RIGHTPAR	308


#line 143 "/home/bonet/tools/lib/bison.h"
 /* #defines token */
/* after #define tokens, before const tokens S5*/
#else
#ifndef YY_BTL_Parser_CLASS
#define YY_BTL_Parser_CLASS BTL_Parser
#endif

#ifndef YY_BTL_Parser_INHERIT
#define YY_BTL_Parser_INHERIT
#endif
#ifndef YY_BTL_Parser_MEMBERS
#define YY_BTL_Parser_MEMBERS 
#endif
#ifndef YY_BTL_Parser_LEX_BODY
#define YY_BTL_Parser_LEX_BODY  
#endif
#ifndef YY_BTL_Parser_ERROR_BODY
#define YY_BTL_Parser_ERROR_BODY  
#endif
#ifndef YY_BTL_Parser_CONSTRUCTOR_PARAM
#define YY_BTL_Parser_CONSTRUCTOR_PARAM
#endif
/* choose between enum and const */
#ifndef YY_BTL_Parser_USE_CONST_TOKEN
#define YY_BTL_Parser_USE_CONST_TOKEN 0
/* yes enum is more compatible with flex,  */
/* so by default we use it */ 
#endif
#if YY_BTL_Parser_USE_CONST_TOKEN != 0
#ifndef YY_BTL_Parser_ENUM_TOKEN
#define YY_BTL_Parser_ENUM_TOKEN yy_BTL_Parser_enum_token
#endif
#endif

class YY_BTL_Parser_CLASS YY_BTL_Parser_INHERIT
{
public: 
#if YY_BTL_Parser_USE_CONST_TOKEN != 0
/* static const int token ... */

/* #line 182 "/home/bonet/tools/lib/bison.h" */
#line 310 "btl.tab.h"
static const int TK_NEW_SYMBOL;
static const int TK_INPUT_PARAMETER_SYMBOL;
static const int TK_PARAMETER_SYMBOL;
static const int TK_NEW_VAR_SYMBOL;
static const int TK_VAR_SYMBOL;
static const int TK_DEF_VAR_SYMBOL;
static const int TK_ACTION_SYMBOL;
static const int TK_INTEGER;
static const int KW_BEGIN_INPUT_PARAMETERS;
static const int KW_END_INPUT_PARAMETERS;
static const int KW_BEGIN_VARIABLES;
static const int KW_END_VARIABLES;
static const int KW_BEGIN_DEFINED_VARIABLES;
static const int KW_END_DEFINED_VARIABLES;
static const int KW_BEGIN_ACTIONS;
static const int KW_END_ACTIONS;
static const int KW_BEGIN_AXIOMS;
static const int KW_END_AXIOMS;
static const int KW_BEGIN_OBSERVABLES;
static const int KW_END_OBSERVABLES;
static const int KW_BEGIN_INITIAL_BELIEF;
static const int KW_END_INITIAL_BELIEF;
static const int KW_BOOLEAN;
static const int KW_INTEGER;
static const int KW_PAIR;
static const int KW_SUCH;
static const int KW_THAT;
static const int KW_IS;
static const int KW_FOR;
static const int KW_WITH;
static const int KW_PRECONDITION;
static const int KW_IF;
static const int KW_THEN;
static const int KW_AND;
static const int KW_OR;
static const int KW_NOT;
static const int KW_SOME;
static const int KW_TRUE;
static const int KW_FALSE;
static const int KW_PLUS;
static const int KW_MINUS;
static const int KW_MOD;
static const int KW_EQ;
static const int KW_NEQ;
static const int KW_ASSIGN;
static const int KW_COLON;
static const int KW_SEMICOLON;
static const int KW_LEFTSQPAR;
static const int KW_RIGHTSQPAR;
static const int KW_LEFTPAR;
static const int KW_RIGHTPAR;


#line 182 "/home/bonet/tools/lib/bison.h"
 /* decl const */
#else
enum YY_BTL_Parser_ENUM_TOKEN { YY_BTL_Parser_NULL_TOKEN=0

/* #line 185 "/home/bonet/tools/lib/bison.h" */
#line 370 "btl.tab.h"
	,TK_NEW_SYMBOL=258
	,TK_INPUT_PARAMETER_SYMBOL=259
	,TK_PARAMETER_SYMBOL=260
	,TK_NEW_VAR_SYMBOL=261
	,TK_VAR_SYMBOL=262
	,TK_DEF_VAR_SYMBOL=263
	,TK_ACTION_SYMBOL=264
	,TK_INTEGER=265
	,KW_BEGIN_INPUT_PARAMETERS=266
	,KW_END_INPUT_PARAMETERS=267
	,KW_BEGIN_VARIABLES=268
	,KW_END_VARIABLES=269
	,KW_BEGIN_DEFINED_VARIABLES=270
	,KW_END_DEFINED_VARIABLES=271
	,KW_BEGIN_ACTIONS=272
	,KW_END_ACTIONS=273
	,KW_BEGIN_AXIOMS=274
	,KW_END_AXIOMS=275
	,KW_BEGIN_OBSERVABLES=276
	,KW_END_OBSERVABLES=277
	,KW_BEGIN_INITIAL_BELIEF=278
	,KW_END_INITIAL_BELIEF=279
	,KW_BOOLEAN=280
	,KW_INTEGER=281
	,KW_PAIR=282
	,KW_SUCH=283
	,KW_THAT=284
	,KW_IS=285
	,KW_FOR=286
	,KW_WITH=287
	,KW_PRECONDITION=288
	,KW_IF=289
	,KW_THEN=290
	,KW_AND=291
	,KW_OR=292
	,KW_NOT=293
	,KW_SOME=294
	,KW_TRUE=295
	,KW_FALSE=296
	,KW_PLUS=297
	,KW_MINUS=298
	,KW_MOD=299
	,KW_EQ=300
	,KW_NEQ=301
	,KW_ASSIGN=302
	,KW_COLON=303
	,KW_SEMICOLON=304
	,KW_LEFTSQPAR=305
	,KW_RIGHTSQPAR=306
	,KW_LEFTPAR=307
	,KW_RIGHTPAR=308


#line 185 "/home/bonet/tools/lib/bison.h"
 /* enum token */
     }; /* end of enum declaration */
#endif
public:
 int YY_BTL_Parser_PARSE(YY_BTL_Parser_PARSE_PARAM);
 virtual void YY_BTL_Parser_ERROR(char *msg) YY_BTL_Parser_ERROR_BODY;
#ifdef YY_BTL_Parser_PURE
#ifdef YY_BTL_Parser_LSP_NEEDED
 virtual int  YY_BTL_Parser_LEX(YY_BTL_Parser_STYPE *YY_BTL_Parser_LVAL,YY_BTL_Parser_LTYPE *YY_BTL_Parser_LLOC) YY_BTL_Parser_LEX_BODY;
#else
 virtual int  YY_BTL_Parser_LEX(YY_BTL_Parser_STYPE *YY_BTL_Parser_LVAL) YY_BTL_Parser_LEX_BODY;
#endif
#else
 virtual int YY_BTL_Parser_LEX() YY_BTL_Parser_LEX_BODY;
 YY_BTL_Parser_STYPE YY_BTL_Parser_LVAL;
#ifdef YY_BTL_Parser_LSP_NEEDED
 YY_BTL_Parser_LTYPE YY_BTL_Parser_LLOC;
#endif
 int YY_BTL_Parser_NERRS;
 int YY_BTL_Parser_CHAR;
#endif
#if YY_BTL_Parser_DEBUG != 0
public:
 int YY_BTL_Parser_DEBUG_FLAG;	/*  nonzero means print parse trace	*/
#endif
public:
 YY_BTL_Parser_CLASS(YY_BTL_Parser_CONSTRUCTOR_PARAM);
public:
 YY_BTL_Parser_MEMBERS 
};
/* other declare folow */
#endif


#if YY_BTL_Parser_COMPATIBILITY != 0
/* backward compatibility */
#ifndef YYSTYPE
#define YYSTYPE YY_BTL_Parser_STYPE
#endif

#ifndef YYLTYPE
#define YYLTYPE YY_BTL_Parser_LTYPE
#endif
#ifndef YYDEBUG
#ifdef YY_BTL_Parser_DEBUG 
#define YYDEBUG YY_BTL_Parser_DEBUG
#endif
#endif

#endif
/* END */

/* #line 236 "/home/bonet/tools/lib/bison.h" */
#line 478 "btl.tab.h"
#endif
