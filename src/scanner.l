/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2017-2018)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *      Stéphane Conversy <stephane.conversy@enac.fr>
 *
 */

%{
  #include <iostream>
  #include <cstdlib>
  #include <algorithm>
  #include "scanner.h"
  #include "driver.h"
  #include "parser.hpp"
  #include "location.hh"

  #define yyterminate() Smala::Parser::make_END (loc);
  //#define YY_USER_ACTION m_driver.increaseLocation (yytext, yyleng);

  using namespace std;
  
  string
  str_filter(const char* s) {
    string src (s);
    src.erase (remove (src.begin (), src.end (), '\\'), src.end ());
    return src;
  }

  string
  hexa_filter(const char *s) {
    string src (s);
    src.replace (0, 1, "x").insert(0, "0");
    return src;
  }

  // use EOL as delimiter outside expression
  static bool _exp_mode = false;
  void lexer_expression_mode_on()
  {
    _exp_mode = true;
  }
  void lexer_expression_mode_off()
  {
    _exp_mode = false;
  }

%}

%option yylineno
%option noyywrap nounput noinput batch debug
%option c++
%option yyclass="Scanner"
%option prefix="Smala_"

NAME [a-zA-Z_\^]((([a-zA-Z0-9_])|("\\-"))*)
INT [0-9]+
ACTION ("run"|"stop"|"dump"|"delete"|"XMLSerialize")
BREAK ("break"|"continue"|"return")
NATIVE_CODE "%{"([^%]|%+[^%}])*%+"}"



%{
  // for *new*: see "A Complete C++ Example": https://www.gnu.org/software/bison/manual/html_node/A-Complete-C_002b_002b-Example.html
  // Code run each time a pattern is matched.
  # define YY_USER_ACTION  { loc.columns (yyleng) /*new*/;  m_driver.increaseLocation (yytext, yyleng) /*old*/; }
%}

%%

%{
  // A handy shortcut to the location held by the driver.
  Smala::location& loc = m_driver.m_loc;
  // Code run each time yylex is called.
  loc.step ();
%}



"/*"([^*]|\*+[^*/])*\*+"/" { loc.lines (std::count(yytext, yytext+yyleng, '\n')); }
"//"[^\n]*            {}
"$"                   { return Smala::Parser::make_DOLLAR (loc);}
";"                   { return Smala::Parser::make_SEMICOLON (loc); }
"{"                   { return Smala::Parser::make_LCB (loc); }
"}"                   { return Smala::Parser::make_RCB (loc); }
"["                   { return Smala::Parser::make_LB (loc); }
"]"                   { return Smala::Parser::make_RB (loc); }
"("                   { return Smala::Parser::make_LP (loc); }
")"                   { return Smala::Parser::make_RP (loc); }
"."                   { return Smala::Parser::make_DOT (loc); }
","                   { return Smala::Parser::make_COMMA (loc); }
"|->"                 { return Smala::Parser::make_ACTIVATOR (loc); }
"->"                  { return Smala::Parser::make_ARROW (loc); }
"!->"                 { return Smala::Parser::make_NOT_ARROW (loc); }
"!->!"                { return Smala::Parser::make_NOT_ARROW_NOT (loc); }
"->!"                 { return Smala::Parser::make_ARROW_NOT (loc); }
"~>"                  { return Smala::Parser::make_CAUSAL_DEP (loc); }
"+"                   { return Smala::Parser::make_PLUS (loc); }
"++"                  { return Smala::Parser::make_INCR (loc); }
"--"                  { return Smala::Parser::make_DECR (loc); }
"-"                   { return Smala::Parser::make_MINUS (loc); }
"*"                   { return Smala::Parser::make_TIMES (loc); }
"/"                   { return Smala::Parser::make_DIVIDE (loc); }
"?"                   { return Smala::Parser::make_QUESTION_MARK (loc); }
"!"                   { return Smala::Parser::make_NOT (loc); }
":"                   { return Smala::Parser::make_COLON (loc); }
"=>"                  { return Smala::Parser::make_CONNECTOR (loc); }
"=:>"                 { return Smala::Parser::make_ASSGNT_CONN (loc); }
"::>"                 { return Smala::Parser::make_PAUSED_CONNECTOR (loc); }
"=:"                  { return Smala::Parser::make_ASSIGNMENT (loc); }
"::"                  { return Smala::Parser::make_PAUSED_ASSIGNMENT (loc); }
"&&"                  { return Smala::Parser::make_AND (loc); }
"||"                  { return Smala::Parser::make_OR (loc); }
"<"                   { return Smala::Parser::make_LT (loc); }
"<="                  { return Smala::Parser::make_LE (loc); }
">"                   { return Smala::Parser::make_GT (loc); }
">="                  { return Smala::Parser::make_GE (loc); }
"=="                  { return Smala::Parser::make_EQ (loc); }
"="                   { return Smala::Parser::make_SIMPLE_EQ (loc); }
"!="                  { return Smala::Parser::make_NEQ (loc); }
"<<"                  { return Smala::Parser::make_INSERT (loc); }
">>"                  { return Smala::Parser::make_GGT (loc); }
"_keep_"              { return Smala::Parser::make_KEEP (loc); }
"int"                 { return Smala::Parser::make_INT_T (loc); }
"double"              { return Smala::Parser::make_DOUBLE_T (loc); }
"string"              { return Smala::Parser::make_STRING_T (loc); }
"list"                { return Smala::Parser::make_LIST_T (loc); }
"TRUE"                { return Smala::Parser::make_TRUE (yytext, loc); }
"FALSE"               { return Smala::Parser::make_FALSE (yytext, loc); }
"null"                { return Smala::Parser::make_NULL (loc);}
"aka"                 { return Smala::Parser::make_AKA (loc); }
"DashArray"           { return Smala::Parser::make_DASHARRAY (loc); }
"FSM"                 { return Smala::Parser::make_FSM (loc); }
"State"               { return Smala::Parser::make_STATE (loc); }
"Process"             { return Smala::Parser::make_PROCESS (loc); }
"&"                   { return Smala::Parser::make_PROCESS_CAST (loc); }
"NativeAction"        { return Smala::Parser::make_NATIVE (loc); }
"NativeCollectionAction" { return Smala::Parser::make_NATIVE_COLLECTION (loc); }
"NativeAsyncAction"   { return Smala::Parser::make_NATIVE_ASYNC (loc); }
"_main_"              { return Smala::Parser::make_MAIN (loc); }
"_define_"            { return Smala::Parser::make_DEFINE (loc); }
"_action_"            { return Smala::Parser::make_NATIVE_ACTION (loc); }
"_native_code_"       { return Smala::Parser::make_NATIVE_CODE (loc); }
{ACTION}              { return Smala::Parser::make_ACTION (yytext, loc); }
{BREAK}               { return Smala::Parser::make_BREAK (yytext, loc); }
"use"                 { return Smala::Parser::make_USE (loc); }
"import"              { return Smala::Parser::make_IMPORT (loc); }
"merge"               { return Smala::Parser::make_MERGE (loc); }
"remove"              { return Smala::Parser::make_REMOVE (loc); }
"moveChild"           { return Smala::Parser::make_MOVE (loc); }
"with"                { return Smala::Parser::make_WITH (loc); }
"from"                { return Smala::Parser::make_FROM (loc); }
"addChildrenTo"       { return Smala::Parser::make_ADD_CHILDREN_TO (loc); }
"toString"            { return Smala::Parser::make_TO_STRING (loc); }
"if"                  { return Smala::Parser::make_IF (loc); }
"else"                { return Smala::Parser::make_ELSE (loc); }
"for"                 { return Smala::Parser::make_FOR (loc); }
"while"               { return Smala::Parser::make_WHILE (loc); }
"print"               { return Smala::Parser::make_PRINT (loc); }
{NATIVE_CODE}         { loc.lines (std::count(yytext, yytext+yyleng, '\n')); return Smala::Parser::make_CODE (yytext, loc); }
{INT}                 { return Smala::Parser::make_INT (yytext, loc); }
"#"[0-9A-Fa-f]+       { return Smala::Parser::make_INT (hexa_filter(yytext), loc); }
"0x"[0-9A-Fa-f]+      { return Smala::Parser::make_INT (hexa_filter(&yytext[1]), loc); }
{INT}"."{INT}         { return Smala::Parser::make_DOUBLE (yytext, loc); }
{INT}"."              { return Smala::Parser::make_DOUBLE (yytext, loc); }
{NAME}         		    { return Smala::Parser::make_NAME (str_filter (yytext), loc); }
\"(\\.|[^\"])*\"      { return Smala::Parser::make_STRING (yytext, loc); }
[\t \.]+              { loc.step (); }
"\\\n"                {}
"\n"+                 { loc.lines (yyleng); loc.step (); if(_exp_mode) return Smala::Parser::make_EOL (loc); }
<<EOF>>               { return yyterminate (); }
%%
