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
  #include "scanner.h"
  #include "driver.h"
  #include "parser.hpp"
  #include "location.hh"

  #define yyterminate() Smala::Parser::make_END (Smala::location());
  #define YY_USER_ACTION m_driver.increaseLocation (yytext, yyleng);

  using namespace std;
  
    string
    str_filter(const char* s) {
      string src (s);
      src.erase (remove (src.begin (), src.end (), '\\'), src.end ());
      return src;
    }
%}

%option yylineno
%option noyywrap
%option c++
%option yyclass="Scanner"
%option prefix="Smala_"

NAME [a-zA-Z_\^]((([a-zA-Z0-9_])|("\\-"))*)
INT [0-9]+
POLY ("Polygon"|"Polyline")
PATH ("Path"|"PathClip")
PATH_POINT ("PathArc"|"PathClosure"|"PathCubic"|"PathLine"|"PathQuadratic"|"PathSubpath"|"PathMove")
GRADIENT ("LinearGradient"|"RadialGradient")
ACTION ("run"|"stop"|"dump"|"notify"|"delete"|"XMLSerialize")
NATIVE_CODE "%{"([^%]|%+[^%}])*%+"}"
%%


"/*"([^*]|\*+[^*/])*\*+"/"            { }
"//"[^\n]*                { }
"$"                   { return Smala::Parser::make_DOLLAR (Smala::location ());}
";"                   { return Smala::Parser::make_SEMICOLON (Smala::location ()); }
"{"                   { return Smala::Parser::make_LCB (Smala::location ()); }
"}"                   { return Smala::Parser::make_RCB (Smala::location ()); }
"["                   { return Smala::Parser::make_LB (Smala::location ()); }
"]"                   { return Smala::Parser::make_RB (Smala::location ()); }
"("                   { return Smala::Parser::make_LP (Smala::location ()); }
")"                   { return Smala::Parser::make_RP (Smala::location ()); }
","                   { return Smala::Parser::make_COMMA (Smala::location ()); }
"->"                  { return Smala::Parser::make_ARROW (Smala::location ()); }
"!->"                 { return Smala::Parser::make_NOT_ARROW (Smala::location ()); }
"!->!"                { return Smala::Parser::make_NOT_ARROW_NOT (Smala::location ()); }
"->!"                 { return Smala::Parser::make_ARROW_NOT (Smala::location ()); }
"+"                   { return Smala::Parser::make_PLUS (Smala::location ()); }
"++"                  { return Smala::Parser::make_INCR (Smala::location ()); }
"--"                  { return Smala::Parser::make_DECR (Smala::location ()); }
"-"                   { return Smala::Parser::make_MINUS (Smala::location ()); }
"*"                   { return Smala::Parser::make_TIMES (Smala::location ()); }
"/"                   { return Smala::Parser::make_DIVIDE (Smala::location ()); }
"?"                   { return Smala::Parser::make_QUESTION_MARK (Smala::location ()); }
"!"                   { return Smala::Parser::make_NOT (Smala::location ()); }
":"                   { return Smala::Parser::make_COLON (Smala::location ()); }
"=>"                  { return Smala::Parser::make_CONNECTOR (Smala::location ()); }
"=:>"                 { return Smala::Parser::make_ASSGNT_CONN (Smala::location ()); }
"::>"                 { return Smala::Parser::make_PAUSED_CONNECTOR (Smala::location ()); }
"=:"                  { return Smala::Parser::make_ASSIGNMENT (Smala::location ()); }
"::"                  { return Smala::Parser::make_PAUSED_ASSIGNMENT (Smala::location ()); }
"&&"                  { return Smala::Parser::make_AND (Smala::location ()); }
"||"                  { return Smala::Parser::make_OR (Smala::location ()); }
"<"                   { return Smala::Parser::make_LT (Smala::location ()); }
"<="                  { return Smala::Parser::make_LE (Smala::location ()); }
">"                   { return Smala::Parser::make_GT (Smala::location ()); }
">="                  { return Smala::Parser::make_GE (Smala::location ()); }
"=="                  { return Smala::Parser::make_EQ (Smala::location ()); }
"="                   { return Smala::Parser::make_SIMPLE_EQ (Smala::location ()); }
"!="                  { return Smala::Parser::make_NEQ (Smala::location ()); }
"<<"                  { return Smala::Parser::make_INSERT (Smala::location ()); }
">>"                  { return Smala::Parser::make_GGT (Smala::location ()); }
"int"                 { return Smala::Parser::make_INT_T (Smala::location ()); }
"double"              { return Smala::Parser::make_DOUBLE_T (Smala::location ()); }
"string"              { return Smala::Parser::make_STRING_T (Smala::location ()); }
"TRUE"                { return Smala::Parser::make_TRUE (yytext, Smala::location ()); }
"FALSE"               { return Smala::Parser::make_FALSE (yytext, Smala::location ()); }
"null"                { return Smala::Parser::make_NULL (Smala::location ());}
"CCall"|"NativeCall"  { return Smala::Parser::make_NATIVE_CALL (Smala::location ()); }
"aka"                 { return Smala::Parser::make_AKA (Smala::location ()); }
"DashArray"           { return Smala::Parser::make_DASHARRAY (Smala::location()); }
"FSM"                 { return Smala::Parser::make_FSM (Smala::location()); }
"State"               { return Smala::Parser::make_STATE (Smala::location()); }
"Process"             { return Smala::Parser::make_PROCESS (Smala::location()); }
"isString"            { return Smala::Parser::make_STRING_CAST (Smala::location()); }
"&"                   { return Smala::Parser::make_PROCESS_CAST (Smala::location()); }
"NativeAction"        { return Smala::Parser::make_NATIVE (Smala::location()); }
"Alias"               { return Smala::Parser::make_ALIAS (Smala::location()); }
"_main_"              { return Smala::Parser::make_MAIN (Smala::location ()); }
"_define_"            { return Smala::Parser::make_DEFINE (Smala::location ()); }
"_action_"            { return Smala::Parser::make_NATIVE_ACTION (Smala::location ()); }
"_native_code_"       { return Smala::Parser::make_NATIVE_CODE (Smala::location ()); }
"_native_java_"       { return Smala::Parser::make_NATIVE_JAVA (Smala::location ()); }
{ACTION}              { return Smala::Parser::make_ACTION (yytext, Smala::location ()); }
"use"                 { return Smala::Parser::make_USE (Smala::location ()); }
"import"              { return Smala::Parser::make_IMPORT (Smala::location ()); }
"merge"               { return Smala::Parser::make_MERGE (Smala::location ()); }
"remove"              { return Smala::Parser::make_REMOVE (Smala::location ()); }
"moveChild"           { return Smala::Parser::make_MOVE (Smala::location ()); }
"with"                { return Smala::Parser::make_WITH (Smala::location ()); }
"from"                { return Smala::Parser::make_FROM (Smala::location ()); }
"addChild"            { return Smala::Parser::make_ADD_CHILD (Smala::location ()); }
"addChildrenTo"       { return Smala::Parser::make_ADD_CHILDREN_TO (Smala::location ()); }
"clone"               { return Smala::Parser::make_CLONE (Smala::location ()); }
"repeat"              { return Smala::Parser::make_REPEAT (Smala::location ()); }
"doubleToString"      { return Smala::Parser::make_DOUBLE_TO_STRING (Smala::location ()); }
"intToString"         { return Smala::Parser::make_INT_TO_STRING (Smala::location ()); }
"if"                  { return Smala::Parser::make_IF (Smala::location ()); }
"then"                { return Smala::Parser::make_THEN (Smala::location ()); }
"else"                { return Smala::Parser::make_ELSE (Smala::location ()); }
"for"                 { return Smala::Parser::make_FOR (Smala::location ()); }
"while"               { return Smala::Parser::make_WHILE (Smala::location ()); }
"print"               { return Smala::Parser::make_PRINT (Smala::location ()); }
{NATIVE_CODE}         { return Smala::Parser::make_CODE (yytext, Smala::location ()); }
{INT}                   { return Smala::Parser::make_INT (yytext, Smala::location ()); }
{INT}"."{INT}           { return Smala::Parser::make_DOUBLE (yytext, Smala::location ()); }
\"(\\.|[^\"])*\"                  { return Smala::Parser::make_STRING (yytext, Smala::location ()); }
{NAME}(("."({NAME}|{INT}))*)      { return Smala::Parser::make_NAME_OR_PATH (str_filter (yytext), Smala::location ()); }
{NAME}(("."({NAME}|{INT}|("["({NAME}|{INT})"]")))*)      { return Smala::Parser::make_NAME_OR_PATH (str_filter (yytext), Smala::location ()); }
{NAME}((("."({NAME}|{INT}))|("::"({NAME}|{INT})))*) { return Smala::Parser::make_NAME_OR_PATH (str_filter (yytext), Smala::location ()); }
{NAME}((("."({NAME}|{INT}))|("."\$({NAME}|{INT})))*) { return Smala::Parser::make_NAME_OR_PATH (str_filter (yytext), Smala::location ()); }
{NAME}":"{NAME}(("."({NAME}|{INT}))*)   { return Smala::Parser::make_URI (str_filter (yytext), Smala::location ()); }
<<EOF>>               { return yyterminate (); }
[\t \.]+             {}
[\n]                  {}
%%

////("+"|"-")?{INT}                   { return Smala::Parser::make_INT (yytext, Smala::location ()); }
//("+"|"-")?{INT}"."{INT}           { return Smala::Parser::make_DOUBLE (yytext, Smala::location ()); }

