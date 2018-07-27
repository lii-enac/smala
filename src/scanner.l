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

NAME [$a-zA-Z_\^]((([a-zA-Z0-9_])|("\\-"))*)
INT [0-9]+
CONTAINER ("Set"|"List")
POLY ("Polygon"|"Polyline")
PATH ("Path"|"PathClip")
PATH_POINT ("PathArc"|"PathClosure"|"PathCubic"|"PathLine"|"PathQuadratic"|"PathSubpath"|"PathMove")
GRADIENT ("LinearGradient"|"RadialGradient")
ACTION ("run"|"stop"|"dump"|"notify"|"delete"|"XMLSerialize")
NATIVE_CODE "%{"([^%]|%+[^%}])*%+"}"
%%


"/*"([^*]|\*+[^*/])*\*+"/"            { }
"//"[^\n]*                { }
"{"                   { return Smala::Parser::make_LCB (Smala::location ()); }
"}"                   { return Smala::Parser::make_RCB (Smala::location ()); }
"("                   { return Smala::Parser::make_LP (Smala::location ()); }
")"                   { return Smala::Parser::make_RP (Smala::location ()); }
","                   { return Smala::Parser::make_COMMA (Smala::location ()); }
"->"                  { return Smala::Parser::make_ARROW (Smala::location ()); }
"+"                   { return Smala::Parser::make_PLUS (Smala::location ()); }
"-"                   { return Smala::Parser::make_MINUS (Smala::location ()); }
"*"                   { return Smala::Parser::make_TIMES (Smala::location ()); }
"/"                   { return Smala::Parser::make_DIVIDE (Smala::location ()); }
"?"                   { return Smala::Parser::make_QUESTION_MARK (Smala::location ()); }
"!"                   { return Smala::Parser::make_NOT (Smala::location ()); }
":"                   { return Smala::Parser::make_COLON (Smala::location ()); }
"=>"                  { return Smala::Parser::make_CONNECTOR (Smala::location ()); }
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
"int"                 { return Smala::Parser::make_INT_T (Smala::location ()); }
"double"              { return Smala::Parser::make_DOUBLE_T (Smala::location ()); }
"string"              { return Smala::Parser::make_STRING_T (Smala::location ()); }
"TRUE"                { return Smala::Parser::make_TRUE (yytext, Smala::location ()); }
"FALSE"               { return Smala::Parser::make_FALSE (yytext, Smala::location ()); }
"null"                { return Smala::Parser::make_NULL (Smala::location ());}
"CCall"|"NativeCall"  { return Smala::Parser::make_NATIVE_CALL (Smala::location ()); }
"aka"                 { return Smala::Parser::make_AKA (Smala::location ()); }
"Assignment"          { return Smala::Parser::make_ASSIGNMENT_CPNT (Smala::location()); }
"AssignmentSequence"  { return Smala::Parser::make_ASSIGNMENT_SEQUENCE (Smala::location()); }
"Binding"             { return Smala::Parser::make_BINDING_CPNT (Smala::location()); }
"Connector"           { return Smala::Parser::make_CONNECTOR_CPNT (Smala::location()); }
"DashArray"           { return Smala::Parser::make_DASHARRAY (Smala::location()); }
"Switch"              { return Smala::Parser::make_SWITCH (Smala::location()); }
"FSM"                 { return Smala::Parser::make_FSM (Smala::location()); }
"State"               { return Smala::Parser::make_STATE (Smala::location()); }
"PixmapCache"         { return Smala::Parser::make_PIXMAP_CACHE (Smala::location()); }
"Component"           { return Smala::Parser::make_COMPONENT (Smala::location()); }
"Process"             { return Smala::Parser::make_PROCESS (Smala::location()); }
"String"              { return Smala::Parser::make_STRING_CPNT (Smala::location()); }
{CONTAINER}           { return Smala::Parser::make_CONTAINER (yytext, Smala::location ()); }
{POLY}                { return Smala::Parser::make_POLY (yytext, Smala::location ()); }
{PATH}                { return Smala::Parser::make_PATH (yytext, Smala::location ()); }
{PATH_POINT}          { return Smala::Parser::make_PATH_POINT (yytext, Smala::location ()); }
{GRADIENT}            { return Smala::Parser::make_GRADIENT (yytext, Smala::location ()); }
"GradientStop"        { return Smala::Parser::make_GRADIENT_STOP (Smala::location ()); }
"Point"               { return Smala::Parser::make_POINT (Smala::location ()); }
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
"with"                { return Smala::Parser::make_WITH (Smala::location ()); }
"from"                { return Smala::Parser::make_FROM (Smala::location ()); }
"addChild"            { return Smala::Parser::make_ADD_CHILD (Smala::location ()); }
"addChildrenTo"       { return Smala::Parser::make_ADD_CHILDREN_TO (Smala::location ()); }
"loadFromXML"         { return Smala::Parser::make_LOAD_XML (Smala::location ()); }
"find"                { return Smala::Parser::make_FIND (Smala::location ()); }
"clone"               { return Smala::Parser::make_CLONE (Smala::location ()); }
"getInt"              { return Smala::Parser::make_GET_INT (Smala::location ()); }
"getDouble"           { return Smala::Parser::make_GET_DOUBLE (Smala::location ()); }
"getBool"             { return Smala::Parser::make_GET_BOOL (Smala::location ()); }
"getString"           { return Smala::Parser::make_GET_STRING (Smala::location ()); }
"getRef"              { return Smala::Parser::make_GET_REF (Smala::location ()); }
{NATIVE_CODE}         { return Smala::Parser::make_CODE (yytext, Smala::location ()); }
{INT}                   { return Smala::Parser::make_INT (yytext, Smala::location ()); }
{INT}"."{INT}           { return Smala::Parser::make_DOUBLE (yytext, Smala::location ()); }
\"(\\.|[^\"])*\"                  { return Smala::Parser::make_STRING (yytext, Smala::location ()); }
{NAME}(("."({NAME}|{INT}))*)      { return Smala::Parser::make_NAME_OR_PATH (str_filter (yytext), Smala::location ()); }
{NAME}":"{NAME}(("."({NAME}|{INT}))*)   { return Smala::Parser::make_URI (str_filter (yytext), Smala::location ()); }
<<EOF>>               { return yyterminate (); }
[\t \n;\.]+             {}
%%

////("+"|"-")?{INT}                   { return Smala::Parser::make_INT (yytext, Smala::location ()); }
//("+"|"-")?{INT}"."{INT}           { return Smala::Parser::make_DOUBLE (yytext, Smala::location ()); }

