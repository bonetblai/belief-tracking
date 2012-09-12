#define YY_BTL_Parser_h_included

/*  A Bison++ parser, made from btl.y  */

 /* with Bison++ version bison++ Version 1.21-8, adapted from GNU bison by coetmeur@icdc.fr
  */


#line 1 "/home/bonet/tools/lib/bison.cc"
/* -*-C-*-  Note some compilers choke on comments on `#line' lines.  */
/* Skeleton output parser for bison,
   Copyright (C) 1984, 1989, 1990 Bob Corbett and Richard Stallman

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 1, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.  */

/* HEADER SECTION */
#if defined( _MSDOS ) || defined(MSDOS) || defined(__MSDOS__) 
#define __MSDOS_AND_ALIKE
#endif
#if defined(_WINDOWS) && defined(_MSC_VER)
#define __HAVE_NO_ALLOCA
#define __MSDOS_AND_ALIKE
#endif

#ifndef alloca
#if defined( __GNUC__)
#define alloca __builtin_alloca

#elif (!defined (__STDC__) && defined (sparc)) || defined (__sparc__) || defined (__sparc)  || defined (__sgi)
#include <alloca.h>

#elif defined (__MSDOS_AND_ALIKE)
#include <malloc.h>
#ifndef __TURBOC__
/* MS C runtime lib */
#define alloca _alloca
#endif

#elif defined(_AIX)
#include <malloc.h>
#pragma alloca

#elif defined(__hpux)
#ifdef __cplusplus
extern "C" {
void *alloca (unsigned int);
};
#else /* not __cplusplus */
void *alloca ();
#endif /* not __cplusplus */

#endif /* not _AIX  not MSDOS, or __TURBOC__ or _AIX, not sparc.  */
#endif /* alloca not defined.  */
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
#ifndef __STDC__
#define const
#endif
#endif
#include <stdio.h>
#define YYBISON 1  

/* #line 73 "/home/bonet/tools/lib/bison.cc" */
#line 85 "btl.tab.c"
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

#line 73 "/home/bonet/tools/lib/bison.cc"
/* %{ and %header{ and %union, during decl */
#define YY_BTL_Parser_BISON 1
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
#endif
#endif
#ifdef YYSTYPE
#ifndef YY_BTL_Parser_STYPE 
#define YY_BTL_Parser_STYPE YYSTYPE
#endif
#endif
#ifdef YYDEBUG
#ifndef YY_BTL_Parser_DEBUG
#define  YY_BTL_Parser_DEBUG YYDEBUG
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

/* #line 117 "/home/bonet/tools/lib/bison.cc" */
#line 173 "btl.tab.c"

#line 117 "/home/bonet/tools/lib/bison.cc"
/*  YY_BTL_Parser_PURE */
#endif

/* section apres lecture def, avant lecture grammaire S2 */

/* #line 121 "/home/bonet/tools/lib/bison.cc" */
#line 182 "btl.tab.c"

#line 121 "/home/bonet/tools/lib/bison.cc"
/* prefix */
#ifndef YY_BTL_Parser_DEBUG

/* #line 123 "/home/bonet/tools/lib/bison.cc" */
#line 189 "btl.tab.c"

#line 123 "/home/bonet/tools/lib/bison.cc"
/* YY_BTL_Parser_DEBUG */
#endif


#ifndef YY_BTL_Parser_LSP_NEEDED

/* #line 128 "/home/bonet/tools/lib/bison.cc" */
#line 199 "btl.tab.c"

#line 128 "/home/bonet/tools/lib/bison.cc"
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
      /* We used to use `unsigned long' as YY_BTL_Parser_STYPE on MSDOS,
	 but it seems better to be consistent.
	 Most programs should declare their own type anyway.  */

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
#if YY_BTL_Parser_COMPATIBILITY != 0
/* backward compatibility */
#ifdef YY_BTL_Parser_LTYPE
#ifndef YYLTYPE
#define YYLTYPE YY_BTL_Parser_LTYPE
#else
/* WARNING obsolete !!! user defined YYLTYPE not reported into generated header */
#endif
#endif
#ifndef YYSTYPE
#define YYSTYPE YY_BTL_Parser_STYPE
#else
/* WARNING obsolete !!! user defined YYSTYPE not reported into generated header */
#endif
#ifdef YY_BTL_Parser_PURE
#ifndef YYPURE
#define YYPURE YY_BTL_Parser_PURE
#endif
#endif
#ifdef YY_BTL_Parser_DEBUG
#ifndef YYDEBUG
#define YYDEBUG YY_BTL_Parser_DEBUG 
#endif
#endif
#ifndef YY_BTL_Parser_ERROR_VERBOSE
#ifdef YYERROR_VERBOSE
#define YY_BTL_Parser_ERROR_VERBOSE YYERROR_VERBOSE
#endif
#endif
#ifndef YY_BTL_Parser_LSP_NEEDED
#ifdef YYLSP_NEEDED
#define YY_BTL_Parser_LSP_NEEDED YYLSP_NEEDED
#endif
#endif
#endif
#ifndef YY_USE_CLASS
/* TOKEN C */

/* #line 236 "/home/bonet/tools/lib/bison.cc" */
#line 312 "btl.tab.c"
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
#define	KW_MOD	297
#define	KW_UNKNOWN	298
#define	KW_DUMMY	299
#define	PLUS	300
#define	MINUS	301
#define	EQ	302
#define	NOT_EQ	303
#define	ASSIGN	304
#define	COLON	305
#define	SEMICOLON	306
#define	LEFTSQPAR	307
#define	RIGHTSQPAR	308
#define	LEFTPAR	309
#define	RIGHTPAR	310


#line 236 "/home/bonet/tools/lib/bison.cc"
 /* #defines tokens */
#else
/* CLASS */
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
#ifndef YY_BTL_Parser_CONSTRUCTOR_CODE
#define YY_BTL_Parser_CONSTRUCTOR_CODE
#endif
#ifndef YY_BTL_Parser_CONSTRUCTOR_INIT
#define YY_BTL_Parser_CONSTRUCTOR_INIT
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

/* #line 280 "/home/bonet/tools/lib/bison.cc" */
#line 415 "btl.tab.c"
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
static const int KW_MOD;
static const int KW_UNKNOWN;
static const int KW_DUMMY;
static const int PLUS;
static const int MINUS;
static const int EQ;
static const int NOT_EQ;
static const int ASSIGN;
static const int COLON;
static const int SEMICOLON;
static const int LEFTSQPAR;
static const int RIGHTSQPAR;
static const int LEFTPAR;
static const int RIGHTPAR;


#line 280 "/home/bonet/tools/lib/bison.cc"
 /* decl const */
#else
enum YY_BTL_Parser_ENUM_TOKEN { YY_BTL_Parser_NULL_TOKEN=0

/* #line 283 "/home/bonet/tools/lib/bison.cc" */
#line 477 "btl.tab.c"
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
	,KW_MOD=297
	,KW_UNKNOWN=298
	,KW_DUMMY=299
	,PLUS=300
	,MINUS=301
	,EQ=302
	,NOT_EQ=303
	,ASSIGN=304
	,COLON=305
	,SEMICOLON=306
	,LEFTSQPAR=307
	,RIGHTSQPAR=308
	,LEFTPAR=309
	,RIGHTPAR=310


#line 283 "/home/bonet/tools/lib/bison.cc"
 /* enum token */
     }; /* end of enum declaration */
#endif
public:
 int YY_BTL_Parser_PARSE (YY_BTL_Parser_PARSE_PARAM);
 virtual void YY_BTL_Parser_ERROR(char *msg) YY_BTL_Parser_ERROR_BODY;
#ifdef YY_BTL_Parser_PURE
#ifdef YY_BTL_Parser_LSP_NEEDED
 virtual int  YY_BTL_Parser_LEX (YY_BTL_Parser_STYPE *YY_BTL_Parser_LVAL,YY_BTL_Parser_LTYPE *YY_BTL_Parser_LLOC) YY_BTL_Parser_LEX_BODY;
#else
 virtual int  YY_BTL_Parser_LEX (YY_BTL_Parser_STYPE *YY_BTL_Parser_LVAL) YY_BTL_Parser_LEX_BODY;
#endif
#else
 virtual int YY_BTL_Parser_LEX() YY_BTL_Parser_LEX_BODY;
 YY_BTL_Parser_STYPE YY_BTL_Parser_LVAL;
#ifdef YY_BTL_Parser_LSP_NEEDED
 YY_BTL_Parser_LTYPE YY_BTL_Parser_LLOC;
#endif
 int   YY_BTL_Parser_NERRS;
 int    YY_BTL_Parser_CHAR;
#endif
#if YY_BTL_Parser_DEBUG != 0
 int YY_BTL_Parser_DEBUG_FLAG;   /*  nonzero means print parse trace     */
#endif
public:
 YY_BTL_Parser_CLASS(YY_BTL_Parser_CONSTRUCTOR_PARAM);
public:
 YY_BTL_Parser_MEMBERS 
};
/* other declare folow */
#if YY_BTL_Parser_USE_CONST_TOKEN != 0

/* #line 314 "/home/bonet/tools/lib/bison.cc" */
#line 567 "btl.tab.c"
const int YY_BTL_Parser_CLASS::TK_NEW_SYMBOL=258;
const int YY_BTL_Parser_CLASS::TK_INPUT_PARAMETER_SYMBOL=259;
const int YY_BTL_Parser_CLASS::TK_PARAMETER_SYMBOL=260;
const int YY_BTL_Parser_CLASS::TK_NEW_VAR_SYMBOL=261;
const int YY_BTL_Parser_CLASS::TK_VAR_SYMBOL=262;
const int YY_BTL_Parser_CLASS::TK_DEF_VAR_SYMBOL=263;
const int YY_BTL_Parser_CLASS::TK_ACTION_SYMBOL=264;
const int YY_BTL_Parser_CLASS::TK_INTEGER=265;
const int YY_BTL_Parser_CLASS::KW_BEGIN_INPUT_PARAMETERS=266;
const int YY_BTL_Parser_CLASS::KW_END_INPUT_PARAMETERS=267;
const int YY_BTL_Parser_CLASS::KW_BEGIN_VARIABLES=268;
const int YY_BTL_Parser_CLASS::KW_END_VARIABLES=269;
const int YY_BTL_Parser_CLASS::KW_BEGIN_DEFINED_VARIABLES=270;
const int YY_BTL_Parser_CLASS::KW_END_DEFINED_VARIABLES=271;
const int YY_BTL_Parser_CLASS::KW_BEGIN_ACTIONS=272;
const int YY_BTL_Parser_CLASS::KW_END_ACTIONS=273;
const int YY_BTL_Parser_CLASS::KW_BEGIN_AXIOMS=274;
const int YY_BTL_Parser_CLASS::KW_END_AXIOMS=275;
const int YY_BTL_Parser_CLASS::KW_BEGIN_OBSERVABLES=276;
const int YY_BTL_Parser_CLASS::KW_END_OBSERVABLES=277;
const int YY_BTL_Parser_CLASS::KW_BEGIN_INITIAL_BELIEF=278;
const int YY_BTL_Parser_CLASS::KW_END_INITIAL_BELIEF=279;
const int YY_BTL_Parser_CLASS::KW_BOOLEAN=280;
const int YY_BTL_Parser_CLASS::KW_INTEGER=281;
const int YY_BTL_Parser_CLASS::KW_PAIR=282;
const int YY_BTL_Parser_CLASS::KW_SUCH=283;
const int YY_BTL_Parser_CLASS::KW_THAT=284;
const int YY_BTL_Parser_CLASS::KW_IS=285;
const int YY_BTL_Parser_CLASS::KW_FOR=286;
const int YY_BTL_Parser_CLASS::KW_WITH=287;
const int YY_BTL_Parser_CLASS::KW_PRECONDITION=288;
const int YY_BTL_Parser_CLASS::KW_IF=289;
const int YY_BTL_Parser_CLASS::KW_THEN=290;
const int YY_BTL_Parser_CLASS::KW_AND=291;
const int YY_BTL_Parser_CLASS::KW_OR=292;
const int YY_BTL_Parser_CLASS::KW_NOT=293;
const int YY_BTL_Parser_CLASS::KW_SOME=294;
const int YY_BTL_Parser_CLASS::KW_TRUE=295;
const int YY_BTL_Parser_CLASS::KW_FALSE=296;
const int YY_BTL_Parser_CLASS::KW_MOD=297;
const int YY_BTL_Parser_CLASS::KW_UNKNOWN=298;
const int YY_BTL_Parser_CLASS::KW_DUMMY=299;
const int YY_BTL_Parser_CLASS::PLUS=300;
const int YY_BTL_Parser_CLASS::MINUS=301;
const int YY_BTL_Parser_CLASS::EQ=302;
const int YY_BTL_Parser_CLASS::NOT_EQ=303;
const int YY_BTL_Parser_CLASS::ASSIGN=304;
const int YY_BTL_Parser_CLASS::COLON=305;
const int YY_BTL_Parser_CLASS::SEMICOLON=306;
const int YY_BTL_Parser_CLASS::LEFTSQPAR=307;
const int YY_BTL_Parser_CLASS::RIGHTSQPAR=308;
const int YY_BTL_Parser_CLASS::LEFTPAR=309;
const int YY_BTL_Parser_CLASS::RIGHTPAR=310;


#line 314 "/home/bonet/tools/lib/bison.cc"
 /* const YY_BTL_Parser_CLASS::token */
#endif
/*apres const  */
YY_BTL_Parser_CLASS::YY_BTL_Parser_CLASS(YY_BTL_Parser_CONSTRUCTOR_PARAM) YY_BTL_Parser_CONSTRUCTOR_INIT
{
#if YY_BTL_Parser_DEBUG != 0
YY_BTL_Parser_DEBUG_FLAG=0;
#endif
YY_BTL_Parser_CONSTRUCTOR_CODE;
};
#endif

/* #line 325 "/home/bonet/tools/lib/bison.cc" */
#line 637 "btl.tab.c"


#define	YYFINAL		158
#define	YYFLAG		-32768
#define	YYNTBASE	56

#define YYTRANSLATE(x) ((unsigned)(x) <= 310 ? yytranslate[x] : 110)

static const char yytranslate[] = {     0,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     1,     2,     3,     4,     5,
     6,     7,     8,     9,    10,    11,    12,    13,    14,    15,
    16,    17,    18,    19,    20,    21,    22,    23,    24,    25,
    26,    27,    28,    29,    30,    31,    32,    33,    34,    35,
    36,    37,    38,    39,    40,    41,    42,    43,    44,    45,
    46,    47,    48,    49,    50,    51,    52,    53,    54,    55
};

#if YY_BTL_Parser_DEBUG != 0
static const short yyprhs[] = {     0,
     0,     8,    12,    13,    16,    18,    23,    25,    27,    31,
    32,    35,    37,    40,    45,    47,    52,    54,    56,    59,
    60,    64,    66,    70,    74,    75,    78,    80,    85,    89,
    90,    93,    95,    98,   102,   105,   109,   110,   113,   114,
   119,   124,   126,   128,   130,   132,   136,   138,   140,   143,
   145,   149,   153,   156,   157,   161,   165,   166,   169,   171,
   175,   179,   180,   183,   185,   188,   191,   195,   196,   199,
   201,   206,   210,   214,   218,   222,   224,   228,   230,   232,
   234,   237,   241,   247,   249,   251,   253,   255,   257,   259,
   261,   263,   265,   267
};

static const short yyrhs[] = {    57,
    61,    70,    73,    89,    92,    95,     0,    11,    58,    12,
     0,     0,    58,    59,     0,    59,     0,   102,    50,    60,
    51,     0,    26,     0,    25,     0,    13,    62,    14,     0,
     0,    62,    63,     0,    63,     0,    64,    51,     0,   104,
    50,    65,    67,     0,    25,     0,    26,    52,    66,    53,
     0,    10,     0,   103,     0,    31,    68,     0,     0,    68,
    36,    69,     0,    69,     0,     5,    30,    65,     0,    15,
    71,    16,     0,     0,    71,    72,     0,    72,     0,   106,
    50,    65,    51,     0,    17,    74,    18,     0,     0,    74,
    75,     0,    75,     0,    76,    79,     0,    77,    78,    51,
     0,   108,    67,     0,    32,    33,    82,     0,     0,    79,
    80,     0,     0,   109,    50,    81,    51,     0,    34,    82,
    35,    87,     0,    83,     0,    84,     0,    40,     0,    41,
     0,    84,    36,    85,     0,    85,     0,    86,     0,    38,
    86,     0,     7,     0,     7,    47,    98,     0,     7,    48,
    98,     0,    87,    88,     0,     0,     7,    49,    98,     0,
    19,    90,    20,     0,     0,    90,    91,     0,    91,     0,
   105,    49,    98,     0,    21,    93,    22,     0,     0,    93,
    94,     0,    94,     0,   105,    51,     0,   107,    51,     0,
    23,    96,    24,     0,     0,    96,    97,     0,    97,     0,
   105,    47,    98,    51,     0,    43,   105,    51,     0,    98,
    45,    99,     0,    98,    46,    99,     0,    98,    37,    99,
     0,    99,     0,    99,    36,   100,     0,   100,     0,   101,
     0,   105,     0,    38,    98,     0,    54,    98,    55,     0,
    54,    98,    55,    42,    10,     0,    41,     0,    40,     0,
    10,     0,     3,     0,     4,     0,     6,     0,     7,     0,
     6,     0,     8,     0,     3,     0,     9,     0
};

#endif

#if YY_BTL_Parser_DEBUG != 0
static const short yyrline[] = { 0,
    88,    93,    95,    98,   100,   103,   107,   109,   112,   114,
   117,   119,   122,   126,   130,   132,   135,   137,   140,   142,
   145,   147,   150,   154,   156,   159,   161,   164,   168,   170,
   173,   175,   178,   182,   186,   190,   192,   195,   197,   200,
   204,   208,   210,   213,   215,   218,   220,   223,   225,   228,
   230,   231,   234,   236,   239,   243,   245,   248,   250,   253,
   257,   259,   262,   264,   267,   269,   272,   274,   277,   279,
   282,   284,   289,   291,   292,   293,   296,   298,   301,   303,
   304,   305,   306,   309,   311,   312,   317,   321,   325,   329,
   333,   337,   341,   345
};

static const char * const yytname[] = {   "$","error","$illegal.","TK_NEW_SYMBOL",
"TK_INPUT_PARAMETER_SYMBOL","TK_PARAMETER_SYMBOL","TK_NEW_VAR_SYMBOL","TK_VAR_SYMBOL",
"TK_DEF_VAR_SYMBOL","TK_ACTION_SYMBOL","TK_INTEGER","KW_BEGIN_INPUT_PARAMETERS",
"KW_END_INPUT_PARAMETERS","KW_BEGIN_VARIABLES","KW_END_VARIABLES","KW_BEGIN_DEFINED_VARIABLES",
"KW_END_DEFINED_VARIABLES","KW_BEGIN_ACTIONS","KW_END_ACTIONS","KW_BEGIN_AXIOMS",
"KW_END_AXIOMS","KW_BEGIN_OBSERVABLES","KW_END_OBSERVABLES","KW_BEGIN_INITIAL_BELIEF",
"KW_END_INITIAL_BELIEF","KW_BOOLEAN","KW_INTEGER","KW_PAIR","KW_SUCH","KW_THAT",
"KW_IS","KW_FOR","KW_WITH","KW_PRECONDITION","KW_IF","KW_THEN","KW_AND","KW_OR",
"KW_NOT","KW_SOME","KW_TRUE","KW_FALSE","KW_MOD","KW_UNKNOWN","KW_DUMMY","PLUS",
"MINUS","EQ","NOT_EQ","ASSIGN","COLON","SEMICOLON","LEFTSQPAR","RIGHTSQPAR",
"LEFTPAR","RIGHTPAR","btl_declarations","btl_input_parameters","input_parameter_list",
"input_parameter_def","primitive_type","btl_variables","variable_list","variable_def",
"new_variable_symbol_with_parameters","fully_qualified_type","bounded_integer",
"parameters_description","parameter_description_list","single_parameter_description",
"btl_defined_variables","defined_variable_list","defined_variable_def","btl_actions",
"action_list","action_def","action_name_and_precondition","action_symbol_with_parameters",
"action_precondition","action_effects","named_conditional_effect","conditional_effect",
"condition","constant_condition","conjunction_condition","literal_condition",
"atomic_condition","effect","atomic_effect","btl_axioms","axiom_list","axiom_def",
"btl_observables","observable_list","observable_def","btl_initial_situation",
"init_list","init_def","expr","term","factor","constant_expr","new_input_parameter_symbol",
"input_parameter_symbol","new_variable_symbol","variable_symbol","new_defined_variable_symbol",
"defined_variable_symbol","new_action_symbol","action_symbol",""
};
#endif

static const short yyr1[] = {     0,
    56,    57,    57,    58,    58,    59,    60,    60,    61,    61,
    62,    62,    63,    64,    65,    65,    66,    66,    67,    67,
    68,    68,    69,    70,    70,    71,    71,    72,    73,    73,
    74,    74,    75,    76,    77,    78,    78,    79,    79,    80,
    81,    82,    82,    83,    83,    84,    84,    85,    85,    86,
    86,    86,    87,    87,    88,    89,    89,    90,    90,    91,
    92,    92,    93,    93,    94,    94,    95,    95,    96,    96,
    97,    97,    98,    98,    98,    98,    99,    99,   100,   100,
   100,   100,   100,   101,   101,   101,   102,   103,   104,   105,
   106,   107,   108,   109
};

static const short yyr2[] = {     0,
     7,     3,     0,     2,     1,     4,     1,     1,     3,     0,
     2,     1,     2,     4,     1,     4,     1,     1,     2,     0,
     3,     1,     3,     3,     0,     2,     1,     4,     3,     0,
     2,     1,     2,     3,     2,     3,     0,     2,     0,     4,
     4,     1,     1,     1,     1,     3,     1,     1,     2,     1,
     3,     3,     2,     0,     3,     3,     0,     2,     1,     3,
     3,     0,     2,     1,     2,     2,     3,     0,     2,     1,
     4,     3,     3,     3,     3,     1,     3,     1,     1,     1,
     2,     3,     5,     1,     1,     1,     1,     1,     1,     1,
     1,     1,     1,     1
};

static const short yydefact[] = {     3,
     0,    10,    87,     0,     5,     0,     0,    25,     2,     4,
     0,    89,     0,    12,     0,     0,     0,    30,     8,     7,
     0,     9,    11,    13,     0,    91,     0,    27,     0,     0,
    57,     6,    15,     0,    20,    24,    26,     0,    93,     0,
    32,    39,    37,    20,     0,    62,     0,     0,    14,     0,
    29,    31,    33,     0,     0,    35,    90,     0,    59,     0,
     0,    68,    88,    17,     0,    18,     0,    19,    22,    28,
    94,    38,     0,     0,    34,    56,    58,     0,    92,     0,
    64,     0,     0,     0,     1,    16,     0,     0,     0,    50,
     0,    44,    45,    36,    42,    43,    47,    48,    86,     0,
    85,    84,     0,    60,    76,    78,    79,    80,    61,    63,
    65,    66,     0,     0,    70,     0,    23,    21,     0,     0,
     0,     0,    49,     0,    81,     0,     0,     0,     0,     0,
     0,    67,    69,     0,     0,    40,    51,    52,    46,    82,
    75,    73,    74,    77,    72,     0,    54,     0,    71,    41,
    83,     0,    53,     0,    55,     0,     0,     0
};

static const short yydefgoto[] = {   156,
     2,     4,     5,    21,     8,    13,    14,    15,    35,    65,
    49,    68,    69,    18,    27,    28,    31,    40,    41,    42,
    43,    55,    53,    72,   120,    94,    95,    96,    97,    98,
   150,   153,    46,    58,    59,    62,    80,    81,    85,   114,
   115,   104,   105,   106,   107,     6,    66,    16,   108,    29,
    83,    44,    73
};

static const short yypact[] = {    -7,
    19,    17,-32768,    16,-32768,   -18,    31,    33,-32768,-32768,
    27,-32768,    41,-32768,    11,    24,    71,    61,-32768,-32768,
    29,-32768,-32768,-32768,    45,-32768,    15,-32768,    32,    76,
    62,-32768,-32768,    34,    52,-32768,-32768,    45,-32768,     8,
-32768,-32768,    53,    52,    77,    66,    57,    83,-32768,    38,
-32768,-32768,    81,    58,    42,-32768,-32768,    13,-32768,    43,
    65,    72,-32768,-32768,    44,-32768,    64,    60,-32768,-32768,
-32768,-32768,    48,     5,-32768,-32768,-32768,     0,-32768,     7,
-32768,    49,    50,    -1,-32768,-32768,    45,    83,    68,    28,
    92,-32768,-32768,-32768,-32768,    67,-32768,-32768,-32768,     0,
-32768,-32768,     0,    14,    69,-32768,-32768,-32768,-32768,-32768,
-32768,-32768,    77,     1,-32768,    59,-32768,-32768,     5,    56,
     0,     0,-32768,    -4,    14,   -28,     0,     0,     0,     0,
    63,-32768,-32768,     0,    73,-32768,    14,    14,-32768,    70,
    69,    69,    69,-32768,-32768,    12,-32768,    94,-32768,   102,
-32768,    74,-32768,     0,    14,   110,   111,-32768
};

static const short yypgoto[] = {-32768,
-32768,-32768,   109,-32768,-32768,-32768,   103,-32768,   -37,-32768,
    75,-32768,    30,-32768,-32768,    88,-32768,-32768,    80,-32768,
-32768,-32768,-32768,-32768,-32768,    -2,-32768,-32768,    -3,    35,
-32768,-32768,-32768,-32768,    78,-32768,-32768,    47,-32768,-32768,
    10,   -98,   -63,    -8,-32768,-32768,-32768,-32768,   -45,-32768,
-32768,-32768,-32768
};


#define	YYLAST		136


static const short yytable[] = {    60,
    50,   125,    90,     1,   126,    57,    57,    57,   127,    99,
    39,    90,    60,    57,    79,    82,   128,   129,     3,    57,
    26,     3,   137,   138,   132,    51,   140,     9,   109,     7,
    36,    11,    76,    91,    82,   146,    12,   100,   116,   101,
   102,   113,    91,   113,    92,    93,    12,    17,   127,   117,
   127,    19,    20,   103,    22,   155,   128,   129,   128,   129,
    63,    24,   149,   141,   142,   143,    64,   131,   116,    33,
    34,    57,    79,    25,   121,   122,    26,    30,    39,    32,
    45,    38,    48,    57,    54,    47,    61,    67,    70,    71,
    74,    78,    75,    87,    84,    88,    86,    89,    90,   111,
   112,   119,   124,   151,   130,   134,   136,   147,   152,   157,
   158,   148,    10,   145,    37,    23,   135,   118,    56,    52,
   139,   144,   154,   133,     0,   123,   110,     0,     0,     0,
     0,     0,     0,     0,     0,    77
};

static const short yycheck[] = {    45,
    38,   100,     7,    11,   103,     7,     7,     7,    37,    10,
     3,     7,    58,     7,     8,    61,    45,    46,     3,     7,
     6,     3,   121,   122,    24,    18,    55,    12,    22,    13,
    16,    50,    20,    38,    80,   134,     6,    38,    84,    40,
    41,    43,    38,    43,    40,    41,     6,    15,    37,    87,
    37,    25,    26,    54,    14,   154,    45,    46,    45,    46,
     4,    51,    51,   127,   128,   129,    10,   113,   114,    25,
    26,     7,     8,    50,    47,    48,     6,    17,     3,    51,
    19,    50,    31,     7,    32,    52,    21,     5,    51,     9,
    33,    49,    51,    30,    23,    36,    53,    50,     7,    51,
    51,    34,    36,    10,    36,    47,    51,    35,     7,     0,
     0,    42,     4,    51,    27,    13,   119,    88,    44,    40,
   124,   130,    49,   114,    -1,    91,    80,    -1,    -1,    -1,
    -1,    -1,    -1,    -1,    -1,    58
};

#line 325 "/home/bonet/tools/lib/bison.cc"
 /* fattrs + tables */

/* parser code folow  */


/* This is the parser code that is written into each bison parser
  when the %semantic_parser declaration is not specified in the grammar.
  It was written by Richard Stallman by simplifying the hairy parser
  used when %semantic_parser is specified.  */

/* Note: dollar marks section change
   the next  is replaced by the list of actions, each action
   as one case of the switch.  */ 

#if YY_BTL_Parser_USE_GOTO != 0
/* 
 SUPRESSION OF GOTO : on some C++ compiler (sun c++)
  the goto is strictly forbidden if any constructor/destructor
  is used in the whole function (very stupid isn't it ?)
 so goto are to be replaced with a 'while/switch/case construct'
 here are the macro to keep some apparent compatibility
*/
#define YYGOTO(lb) {yy_gotostate=lb;continue;}
#define YYBEGINGOTO  enum yy_labels yy_gotostate=yygotostart; \
                     for(;;) switch(yy_gotostate) { case yygotostart: {
#define YYLABEL(lb) } case lb: {
#define YYENDGOTO } } 
#define YYBEGINDECLARELABEL enum yy_labels {yygotostart
#define YYDECLARELABEL(lb) ,lb
#define YYENDDECLARELABEL  };
#else
/* macro to keep goto */
#define YYGOTO(lb) goto lb
#define YYBEGINGOTO 
#define YYLABEL(lb) lb:
#define YYENDGOTO
#define YYBEGINDECLARELABEL 
#define YYDECLARELABEL(lb)
#define YYENDDECLARELABEL 
#endif
/* LABEL DECLARATION */
YYBEGINDECLARELABEL
  YYDECLARELABEL(yynewstate)
  YYDECLARELABEL(yybackup)
/* YYDECLARELABEL(yyresume) */
  YYDECLARELABEL(yydefault)
  YYDECLARELABEL(yyreduce)
  YYDECLARELABEL(yyerrlab)   /* here on detecting error */
  YYDECLARELABEL(yyerrlab1)   /* here on error raised explicitly by an action */
  YYDECLARELABEL(yyerrdefault)  /* current state does not do anything special for the error token. */
  YYDECLARELABEL(yyerrpop)   /* pop the current state because it cannot handle the error token */
  YYDECLARELABEL(yyerrhandle)  
YYENDDECLARELABEL
/* ALLOCA SIMULATION */
/* __HAVE_NO_ALLOCA */
#ifdef __HAVE_NO_ALLOCA
int __alloca_free_ptr(char *ptr,char *ref)
{if(ptr!=ref) free(ptr);
 return 0;}

#define __ALLOCA_alloca(size) malloc(size)
#define __ALLOCA_free(ptr,ref) __alloca_free_ptr((char *)ptr,(char *)ref)

#ifdef YY_BTL_Parser_LSP_NEEDED
#define __ALLOCA_return(num) \
            return( __ALLOCA_free(yyss,yyssa)+\
		    __ALLOCA_free(yyvs,yyvsa)+\
		    __ALLOCA_free(yyls,yylsa)+\
		   (num))
#else
#define __ALLOCA_return(num) \
            return( __ALLOCA_free(yyss,yyssa)+\
		    __ALLOCA_free(yyvs,yyvsa)+\
		   (num))
#endif
#else
#define __ALLOCA_return(num) return(num)
#define __ALLOCA_alloca(size) alloca(size)
#define __ALLOCA_free(ptr,ref) 
#endif

/* ENDALLOCA SIMULATION */

#define yyerrok         (yyerrstatus = 0)
#define yyclearin       (YY_BTL_Parser_CHAR = YYEMPTY)
#define YYEMPTY         -2
#define YYEOF           0
#define YYACCEPT        __ALLOCA_return(0)
#define YYABORT         __ALLOCA_return(1)
#define YYERROR         YYGOTO(yyerrlab1)
/* Like YYERROR except do call yyerror.
   This remains here temporarily to ease the
   transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */
#define YYFAIL          YYGOTO(yyerrlab)
#define YYRECOVERING()  (!!yyerrstatus)
#define YYBACKUP(token, value) \
do                                                              \
  if (YY_BTL_Parser_CHAR == YYEMPTY && yylen == 1)                               \
    { YY_BTL_Parser_CHAR = (token), YY_BTL_Parser_LVAL = (value);                 \
      yychar1 = YYTRANSLATE (YY_BTL_Parser_CHAR);                                \
      YYPOPSTACK;                                               \
      YYGOTO(yybackup);                                            \
    }                                                           \
  else                                                          \
    { YY_BTL_Parser_ERROR ("syntax error: cannot back up"); YYERROR; }   \
while (0)

#define YYTERROR        1
#define YYERRCODE       256

#ifndef YY_BTL_Parser_PURE
/* UNPURE */
#define YYLEX           YY_BTL_Parser_LEX()
#ifndef YY_USE_CLASS
/* If nonreentrant, and not class , generate the variables here */
int     YY_BTL_Parser_CHAR;                      /*  the lookahead symbol        */
YY_BTL_Parser_STYPE      YY_BTL_Parser_LVAL;              /*  the semantic value of the */
				/*  lookahead symbol    */
int YY_BTL_Parser_NERRS;                 /*  number of parse errors so far */
#ifdef YY_BTL_Parser_LSP_NEEDED
YY_BTL_Parser_LTYPE YY_BTL_Parser_LLOC;   /*  location data for the lookahead     */
			/*  symbol                              */
#endif
#endif


#else
/* PURE */
#ifdef YY_BTL_Parser_LSP_NEEDED
#define YYLEX           YY_BTL_Parser_LEX(&YY_BTL_Parser_LVAL, &YY_BTL_Parser_LLOC)
#else
#define YYLEX           YY_BTL_Parser_LEX(&YY_BTL_Parser_LVAL)
#endif
#endif
#ifndef YY_USE_CLASS
#if YY_BTL_Parser_DEBUG != 0
int YY_BTL_Parser_DEBUG_FLAG;                    /*  nonzero means print parse trace     */
/* Since this is uninitialized, it does not stop multiple parsers
   from coexisting.  */
#endif
#endif



/*  YYINITDEPTH indicates the initial size of the parser's stacks       */

#ifndef YYINITDEPTH
#define YYINITDEPTH 200
#endif

/*  YYMAXDEPTH is the maximum size the stacks can grow to
    (effective only if the built-in stack extension method is used).  */

#if YYMAXDEPTH == 0
#undef YYMAXDEPTH
#endif

#ifndef YYMAXDEPTH
#define YYMAXDEPTH 10000
#endif


#if __GNUC__ > 1                /* GNU C and GNU C++ define this.  */
#define __yy_bcopy(FROM,TO,COUNT)       __builtin_memcpy(TO,FROM,COUNT)
#else                           /* not GNU C or C++ */

/* This is the most reliable way to avoid incompatibilities
   in available built-in functions on various systems.  */

#ifdef __cplusplus
static void __yy_bcopy (char *from, char *to, int count)
#else
#ifdef __STDC__
static void __yy_bcopy (char *from, char *to, int count)
#else
static void __yy_bcopy (from, to, count)
     char *from;
     char *to;
     int count;
#endif
#endif
{
  register char *f = from;
  register char *t = to;
  register int i = count;

  while (i-- > 0)
    *t++ = *f++;
}
#endif

int
#ifdef YY_USE_CLASS
 YY_BTL_Parser_CLASS::
#endif
     YY_BTL_Parser_PARSE(YY_BTL_Parser_PARSE_PARAM)
#ifndef __STDC__
#ifndef __cplusplus
#ifndef YY_USE_CLASS
/* parameter definition without protypes */
YY_BTL_Parser_PARSE_PARAM_DEF
#endif
#endif
#endif
{
  register int yystate;
  register int yyn;
  register short *yyssp;
  register YY_BTL_Parser_STYPE *yyvsp;
  int yyerrstatus;      /*  number of tokens to shift before error messages enabled */
  int yychar1=0;          /*  lookahead token as an internal (translated) token number */

  short yyssa[YYINITDEPTH];     /*  the state stack                     */
  YY_BTL_Parser_STYPE yyvsa[YYINITDEPTH];        /*  the semantic value stack            */

  short *yyss = yyssa;          /*  refer to the stacks thru separate pointers */
  YY_BTL_Parser_STYPE *yyvs = yyvsa;     /*  to allow yyoverflow to reallocate them elsewhere */

#ifdef YY_BTL_Parser_LSP_NEEDED
  YY_BTL_Parser_LTYPE yylsa[YYINITDEPTH];        /*  the location stack                  */
  YY_BTL_Parser_LTYPE *yyls = yylsa;
  YY_BTL_Parser_LTYPE *yylsp;

#define YYPOPSTACK   (yyvsp--, yyssp--, yylsp--)
#else
#define YYPOPSTACK   (yyvsp--, yyssp--)
#endif

  int yystacksize = YYINITDEPTH;

#ifdef YY_BTL_Parser_PURE
  int YY_BTL_Parser_CHAR;
  YY_BTL_Parser_STYPE YY_BTL_Parser_LVAL;
  int YY_BTL_Parser_NERRS;
#ifdef YY_BTL_Parser_LSP_NEEDED
  YY_BTL_Parser_LTYPE YY_BTL_Parser_LLOC;
#endif
#endif

  YY_BTL_Parser_STYPE yyval;             /*  the variable used to return         */
				/*  semantic values from the action     */
				/*  routines                            */

  int yylen;
/* start loop, in which YYGOTO may be used. */
YYBEGINGOTO

#if YY_BTL_Parser_DEBUG != 0
  if (YY_BTL_Parser_DEBUG_FLAG)
    fprintf(stderr, "Starting parse\n");
#endif
  yystate = 0;
  yyerrstatus = 0;
  YY_BTL_Parser_NERRS = 0;
  YY_BTL_Parser_CHAR = YYEMPTY;          /* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */

  yyssp = yyss - 1;
  yyvsp = yyvs;
#ifdef YY_BTL_Parser_LSP_NEEDED
  yylsp = yyls;
#endif

/* Push a new state, which is found in  yystate  .  */
/* In all cases, when you get here, the value and location stacks
   have just been pushed. so pushing a state here evens the stacks.  */
YYLABEL(yynewstate)

  *++yyssp = yystate;

  if (yyssp >= yyss + yystacksize - 1)
    {
      /* Give user a chance to reallocate the stack */
      /* Use copies of these so that the &'s don't force the real ones into memory. */
      YY_BTL_Parser_STYPE *yyvs1 = yyvs;
      short *yyss1 = yyss;
#ifdef YY_BTL_Parser_LSP_NEEDED
      YY_BTL_Parser_LTYPE *yyls1 = yyls;
#endif

      /* Get the current used size of the three stacks, in elements.  */
      int size = yyssp - yyss + 1;

#ifdef yyoverflow
      /* Each stack pointer address is followed by the size of
	 the data in use in that stack, in bytes.  */
#ifdef YY_BTL_Parser_LSP_NEEDED
      /* This used to be a conditional around just the two extra args,
	 but that might be undefined if yyoverflow is a macro.  */
      yyoverflow("parser stack overflow",
		 &yyss1, size * sizeof (*yyssp),
		 &yyvs1, size * sizeof (*yyvsp),
		 &yyls1, size * sizeof (*yylsp),
		 &yystacksize);
#else
      yyoverflow("parser stack overflow",
		 &yyss1, size * sizeof (*yyssp),
		 &yyvs1, size * sizeof (*yyvsp),
		 &yystacksize);
#endif

      yyss = yyss1; yyvs = yyvs1;
#ifdef YY_BTL_Parser_LSP_NEEDED
      yyls = yyls1;
#endif
#else /* no yyoverflow */
      /* Extend the stack our own way.  */
      if (yystacksize >= YYMAXDEPTH)
	{
	  YY_BTL_Parser_ERROR("parser stack overflow");
	  __ALLOCA_return(2);
	}
      yystacksize *= 2;
      if (yystacksize > YYMAXDEPTH)
	yystacksize = YYMAXDEPTH;
      yyss = (short *) __ALLOCA_alloca (yystacksize * sizeof (*yyssp));
      __yy_bcopy ((char *)yyss1, (char *)yyss, size * sizeof (*yyssp));
      __ALLOCA_free(yyss1,yyssa);
      yyvs = (YY_BTL_Parser_STYPE *) __ALLOCA_alloca (yystacksize * sizeof (*yyvsp));
      __yy_bcopy ((char *)yyvs1, (char *)yyvs, size * sizeof (*yyvsp));
      __ALLOCA_free(yyvs1,yyvsa);
#ifdef YY_BTL_Parser_LSP_NEEDED
      yyls = (YY_BTL_Parser_LTYPE *) __ALLOCA_alloca (yystacksize * sizeof (*yylsp));
      __yy_bcopy ((char *)yyls1, (char *)yyls, size * sizeof (*yylsp));
      __ALLOCA_free(yyls1,yylsa);
#endif
#endif /* no yyoverflow */

      yyssp = yyss + size - 1;
      yyvsp = yyvs + size - 1;
#ifdef YY_BTL_Parser_LSP_NEEDED
      yylsp = yyls + size - 1;
#endif

#if YY_BTL_Parser_DEBUG != 0
      if (YY_BTL_Parser_DEBUG_FLAG)
	fprintf(stderr, "Stack size increased to %d\n", yystacksize);
#endif

      if (yyssp >= yyss + yystacksize - 1)
	YYABORT;
    }

#if YY_BTL_Parser_DEBUG != 0
  if (YY_BTL_Parser_DEBUG_FLAG)
    fprintf(stderr, "Entering state %d\n", yystate);
#endif

  YYGOTO(yybackup);
YYLABEL(yybackup)

/* Do appropriate processing given the current state.  */
/* Read a lookahead token if we need one and don't already have one.  */
/* YYLABEL(yyresume) */

  /* First try to decide what to do without reference to lookahead token.  */

  yyn = yypact[yystate];
  if (yyn == YYFLAG)
    YYGOTO(yydefault);

  /* Not known => get a lookahead token if don't already have one.  */

  /* yychar is either YYEMPTY or YYEOF
     or a valid token in external form.  */

  if (YY_BTL_Parser_CHAR == YYEMPTY)
    {
#if YY_BTL_Parser_DEBUG != 0
      if (YY_BTL_Parser_DEBUG_FLAG)
	fprintf(stderr, "Reading a token: ");
#endif
      YY_BTL_Parser_CHAR = YYLEX;
    }

  /* Convert token to internal form (in yychar1) for indexing tables with */

  if (YY_BTL_Parser_CHAR <= 0)           /* This means end of input. */
    {
      yychar1 = 0;
      YY_BTL_Parser_CHAR = YYEOF;                /* Don't call YYLEX any more */

#if YY_BTL_Parser_DEBUG != 0
      if (YY_BTL_Parser_DEBUG_FLAG)
	fprintf(stderr, "Now at end of input.\n");
#endif
    }
  else
    {
      yychar1 = YYTRANSLATE(YY_BTL_Parser_CHAR);

#if YY_BTL_Parser_DEBUG != 0
      if (YY_BTL_Parser_DEBUG_FLAG)
	{
	  fprintf (stderr, "Next token is %d (%s", YY_BTL_Parser_CHAR, yytname[yychar1]);
	  /* Give the individual parser a way to print the precise meaning
	     of a token, for further debugging info.  */
#ifdef YYPRINT
	  YYPRINT (stderr, YY_BTL_Parser_CHAR, YY_BTL_Parser_LVAL);
#endif
	  fprintf (stderr, ")\n");
	}
#endif
    }

  yyn += yychar1;
  if (yyn < 0 || yyn > YYLAST || yycheck[yyn] != yychar1)
    YYGOTO(yydefault);

  yyn = yytable[yyn];

  /* yyn is what to do for this token type in this state.
     Negative => reduce, -yyn is rule number.
     Positive => shift, yyn is new state.
       New state is final state => don't bother to shift,
       just return success.
     0, or most negative number => error.  */

  if (yyn < 0)
    {
      if (yyn == YYFLAG)
	YYGOTO(yyerrlab);
      yyn = -yyn;
      YYGOTO(yyreduce);
    }
  else if (yyn == 0)
    YYGOTO(yyerrlab);

  if (yyn == YYFINAL)
    YYACCEPT;

  /* Shift the lookahead token.  */

#if YY_BTL_Parser_DEBUG != 0
  if (YY_BTL_Parser_DEBUG_FLAG)
    fprintf(stderr, "Shifting token %d (%s), ", YY_BTL_Parser_CHAR, yytname[yychar1]);
#endif

  /* Discard the token being shifted unless it is eof.  */
  if (YY_BTL_Parser_CHAR != YYEOF)
    YY_BTL_Parser_CHAR = YYEMPTY;

  *++yyvsp = YY_BTL_Parser_LVAL;
#ifdef YY_BTL_Parser_LSP_NEEDED
  *++yylsp = YY_BTL_Parser_LLOC;
#endif

  /* count tokens shifted since error; after three, turn off error status.  */
  if (yyerrstatus) yyerrstatus--;

  yystate = yyn;
  YYGOTO(yynewstate);

/* Do the default action for the current state.  */
YYLABEL(yydefault)

  yyn = yydefact[yystate];
  if (yyn == 0)
    YYGOTO(yyerrlab);

/* Do a reduction.  yyn is the number of a rule to reduce with.  */
YYLABEL(yyreduce)
  yylen = yyr2[yyn];
  if (yylen > 0)
    yyval = yyvsp[1-yylen]; /* implement default value of the action */

#if YY_BTL_Parser_DEBUG != 0
  if (YY_BTL_Parser_DEBUG_FLAG)
    {
      int i;

      fprintf (stderr, "Reducing via rule %d (line %d), ",
	       yyn, yyrline[yyn]);

      /* Print the symbols being reduced, and their result.  */
      for (i = yyprhs[yyn]; yyrhs[i] > 0; i++)
	fprintf (stderr, "%s ", yytname[yyrhs[i]]);
      fprintf (stderr, " -> %s\n", yytname[yyr1[yyn]]);
    }
#endif


/* #line 811 "/home/bonet/tools/lib/bison.cc" */
#line 1375 "btl.tab.c"

  switch (yyn) {

case 87:
#line 318 "btl.y"
{ ;
    break;}
case 89:
#line 326 "btl.y"
{ ;
    break;}
case 91:
#line 334 "btl.y"
{ ;
    break;}
case 93:
#line 342 "btl.y"
{ ;
    break;}
}

#line 811 "/home/bonet/tools/lib/bison.cc"
   /* the action file gets copied in in place of this dollarsign  */
  yyvsp -= yylen;
  yyssp -= yylen;
#ifdef YY_BTL_Parser_LSP_NEEDED
  yylsp -= yylen;
#endif

#if YY_BTL_Parser_DEBUG != 0
  if (YY_BTL_Parser_DEBUG_FLAG)
    {
      short *ssp1 = yyss - 1;
      fprintf (stderr, "state stack now");
      while (ssp1 != yyssp)
	fprintf (stderr, " %d", *++ssp1);
      fprintf (stderr, "\n");
    }
#endif

  *++yyvsp = yyval;

#ifdef YY_BTL_Parser_LSP_NEEDED
  yylsp++;
  if (yylen == 0)
    {
      yylsp->first_line = YY_BTL_Parser_LLOC.first_line;
      yylsp->first_column = YY_BTL_Parser_LLOC.first_column;
      yylsp->last_line = (yylsp-1)->last_line;
      yylsp->last_column = (yylsp-1)->last_column;
      yylsp->text = 0;
    }
  else
    {
      yylsp->last_line = (yylsp+yylen-1)->last_line;
      yylsp->last_column = (yylsp+yylen-1)->last_column;
    }
#endif

  /* Now "shift" the result of the reduction.
     Determine what state that goes to,
     based on the state we popped back to
     and the rule number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTBASE] + *yyssp;
  if (yystate >= 0 && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTBASE];

  YYGOTO(yynewstate);

YYLABEL(yyerrlab)   /* here on detecting error */

  if (! yyerrstatus)
    /* If not already recovering from an error, report this error.  */
    {
      ++YY_BTL_Parser_NERRS;

#ifdef YY_BTL_Parser_ERROR_VERBOSE
      yyn = yypact[yystate];

      if (yyn > YYFLAG && yyn < YYLAST)
	{
	  int size = 0;
	  char *msg;
	  int x, count;

	  count = 0;
	  /* Start X at -yyn if nec to avoid negative indexes in yycheck.  */
	  for (x = (yyn < 0 ? -yyn : 0);
	       x < (sizeof(yytname) / sizeof(char *)); x++)
	    if (yycheck[x + yyn] == x)
	      size += strlen(yytname[x]) + 15, count++;
	  msg = (char *) malloc(size + 15);
	  if (msg != 0)
	    {
	      strcpy(msg, "parse error");

	      if (count < 5)
		{
		  count = 0;
		  for (x = (yyn < 0 ? -yyn : 0);
		       x < (sizeof(yytname) / sizeof(char *)); x++)
		    if (yycheck[x + yyn] == x)
		      {
			strcat(msg, count == 0 ? ", expecting `" : " or `");
			strcat(msg, yytname[x]);
			strcat(msg, "'");
			count++;
		      }
		}
	      YY_BTL_Parser_ERROR(msg);
	      free(msg);
	    }
	  else
	    YY_BTL_Parser_ERROR ("parse error; also virtual memory exceeded");
	}
      else
#endif /* YY_BTL_Parser_ERROR_VERBOSE */
	YY_BTL_Parser_ERROR("parse error");
    }

  YYGOTO(yyerrlab1);
YYLABEL(yyerrlab1)   /* here on error raised explicitly by an action */

  if (yyerrstatus == 3)
    {
      /* if just tried and failed to reuse lookahead token after an error, discard it.  */

      /* return failure if at end of input */
      if (YY_BTL_Parser_CHAR == YYEOF)
	YYABORT;

#if YY_BTL_Parser_DEBUG != 0
      if (YY_BTL_Parser_DEBUG_FLAG)
	fprintf(stderr, "Discarding token %d (%s).\n", YY_BTL_Parser_CHAR, yytname[yychar1]);
#endif

      YY_BTL_Parser_CHAR = YYEMPTY;
    }

  /* Else will try to reuse lookahead token
     after shifting the error token.  */

  yyerrstatus = 3;              /* Each real token shifted decrements this */

  YYGOTO(yyerrhandle);

YYLABEL(yyerrdefault)  /* current state does not do anything special for the error token. */

#if 0
  /* This is wrong; only states that explicitly want error tokens
     should shift them.  */
  yyn = yydefact[yystate];  /* If its default is to accept any token, ok.  Otherwise pop it.*/
  if (yyn) YYGOTO(yydefault);
#endif

YYLABEL(yyerrpop)   /* pop the current state because it cannot handle the error token */

  if (yyssp == yyss) YYABORT;
  yyvsp--;
  yystate = *--yyssp;
#ifdef YY_BTL_Parser_LSP_NEEDED
  yylsp--;
#endif

#if YY_BTL_Parser_DEBUG != 0
  if (YY_BTL_Parser_DEBUG_FLAG)
    {
      short *ssp1 = yyss - 1;
      fprintf (stderr, "Error: state stack now");
      while (ssp1 != yyssp)
	fprintf (stderr, " %d", *++ssp1);
      fprintf (stderr, "\n");
    }
#endif

YYLABEL(yyerrhandle)

  yyn = yypact[yystate];
  if (yyn == YYFLAG)
    YYGOTO(yyerrdefault);

  yyn += YYTERROR;
  if (yyn < 0 || yyn > YYLAST || yycheck[yyn] != YYTERROR)
    YYGOTO(yyerrdefault);

  yyn = yytable[yyn];
  if (yyn < 0)
    {
      if (yyn == YYFLAG)
	YYGOTO(yyerrpop);
      yyn = -yyn;
      YYGOTO(yyreduce);
    }
  else if (yyn == 0)
    YYGOTO(yyerrpop);

  if (yyn == YYFINAL)
    YYACCEPT;

#if YY_BTL_Parser_DEBUG != 0
  if (YY_BTL_Parser_DEBUG_FLAG)
    fprintf(stderr, "Shifting error token, ");
#endif

  *++yyvsp = YY_BTL_Parser_LVAL;
#ifdef YY_BTL_Parser_LSP_NEEDED
  *++yylsp = YY_BTL_Parser_LLOC;
#endif

  yystate = yyn;
  YYGOTO(yynewstate);
/* end loop, in which YYGOTO may be used. */
  YYENDGOTO
}

/* END */

/* #line 1010 "/home/bonet/tools/lib/bison.cc" */
#line 1599 "btl.tab.c"
#line 349 "btl.y"


