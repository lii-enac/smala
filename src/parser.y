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

%skeleton "lalr1.cc" /* -*- C++ -*- */
%require "3.0"
%defines
%define parser_class_name { Parser }

%define api.token.constructor
%define api.value.type variant
%define parse.assert
%define api.namespace { Smala }
%code requires
{
  #include <iostream>
  #include <string>
  #include <vector>
  #include <stdint.h>
  #include "node.h"
  #include "activator_node.h"
  #include "operator_node.h"
  #include "instruction_node.h"
  #include "binary_instruction_node.h"
  #include "ccall_node.h"
  #include "smala_native.h"
  #include "ctrl_node.h"
  #include "alternative_node.h"
  #include "local_node.h"
  #include "arg_node.h"
  #include "native_action_node.h"
  #include "native_code_node.h"

  using namespace std;

  namespace Smala {
    class Scanner;
    class Driver;
  }
}

%code top
{
  #include <iostream>
  #include "scanner.h"
  #include "parser.hpp"
  #include "driver.h"
  #include "location.hh"
  #include "node.h"

  static Smala::Parser::symbol_type yylex (Smala::Scanner &scanner, Smala::Driver &driver) {
    return scanner.get_next_token ();
  }
    
  using namespace Smala;
  vector<Node*> parent_list;
  vector<Node*> expression;
  int func_num = 0;
}


%code
{

 OperatorNode* make_BINARY_OP (Smala::Driver &driver, const char* opname, Node* left, Node* right)
  {
    OperatorNode *node = new OperatorNode(opname,"");
    node->set_node_type (BINARY_OP);
    node->set_left (left);
    node->set_right (right);
    driver.add_node (node);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    expression.push_back (node);
    return node;
  }


 OperatorNode* make_BINARY_OP_INVERT (Smala::Driver &driver, const char* opname, Node* left, Node* right)
  {
    OperatorNode *node = new OperatorNode(opname,"");
    node->set_node_type (BINARY_OP);
    node->set_left (left);
    node->set_right (right);
    driver.add_node (node);
    expression.push_back (node);
    OperatorNode *node2 = new OperatorNode ("SignInverter", "");
    node2->set_node_type (UNARY_OP);
    node2->set_right (node);
    driver.add_node (node2);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    expression.push_back (node2);
    return node2;
  }

 OperatorNode* make_UNARY_OP (Smala::Driver &driver, const char* opname, Node* right)
  {
    OperatorNode *node = new OperatorNode(opname,"");
    node->set_node_type (UNARY_OP);
    node->set_right (right);
    driver.add_node (node);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    expression.push_back (node);
    return node;
  }

}


%lex-param { Smala::Scanner &scanner }
%lex-param { Smala::Driver &driver }
%parse-param { Smala::Scanner &scanner }
%parse-param { Smala::Driver &driver }

%locations
%define parse.trace
%define parse.error verbose
%define api.token.prefix {TOKEN_}

%token ARROW "->"
%token BINDING_CPNT "Binding"
%token CONNECTOR "=>"
%token PAUSED_CONNECTOR "::>"
%token CONNECTOR_CPNT "Connector"
%token ASSIGNMENT "=:"
%token ASSIGNMENT_SEQUENCE "AssignmentSequence"
%token PAUSED_ASSIGNMENT "::"
%token ASSIGNMENT_CPNT "Assignment"
%token COMPONENT "Component"
%token PROCESS "Process"
%token STRING_CPNT "String"
%token <string> CONTAINER "<container>"
%token <string> POLY "<poly>"
%token <string> PATH "<path>"
%token <string> PATH_POINT "<path-point>"
%token <string> GRADIENT "<gradient>"
%token GRADIENT_STOP "GradientStop"
%token FSM "FSM"
%token STATE "State"
%token POINT "Point"
%token SWITCH "Switch"
%token PIXMAP_CACHE "PixmapCache"
%token MINUS "-"
%token PLUS "+"
%token TIMES "*"
%token DIVIDE "/"
%token COLON ":"
%token QUESTION_MARK "?"
%token AND "&&"
%token OR "||"
%token LT "<"
%token LE "<="
%token GT ">"
%token GE ">="
%token EQ "=="
%token SIMPLE_EQ "="
%token NEQ "!="
%token NOT"!"
%token LCB "{"
%token RCB "}"
%token LP "("
%token RP ")"
%token COMMA ","
%token INSERT "insert"
%token INT_T "int"
%token DOUBLE_T "double"
%token STRING_T "string"
%token USE "use"
%token NULL "null"
%token ADD_CHILD "addChild"
%token ADD_CHILDREN_TO "addChildrenTo"
%token MERGE "merge"
%token REMOVE "remove"
%token WITH "with"
%token FROM "from"
%token LOAD_XML "loadFromXML"
%token FIND "find"
%token NATIVE_CALL "CCal|NativeCall"
%token CLONE "clone"
%token GET_INT "getInt"
%token GET_DOUBLE "getDouble"
%token GET_BOOL "getBool"
%token GET_STRING "getString"
%token GET_REF "getRef"
%token MAIN "_main_"
%token DEFINE "_define_"
%token NATIVE "Native"
%token NATIVE_ACTION "_action_"
%token NATIVE_CODE "_native_code_"
%token NATIVE_JAVA "_native_java_"
%token <string> CODE "<native code>"
%token <string> VERBOSE ";;"
%token ALIAS "Alias"
%token DASHARRAY "DashArray"
%token <string> INT "<int>"
%token <string> DOUBLE "<double>"
%token <string> STRING "<string>"
%token <string> TRUE "TRUE"
%token <string> FALSE "FALSE"
%token <string> ACTION "<action>"
%token AKA "aka"
%token IMPORT "import"
%token END 0 "end of file"
%token <string> NAME_OR_PATH "name_or_path"
%token <string> URI "URI"

%type <int> items
%type <int> assignment_items
%type <int> points
%type <int> path_points
%type <int> gradient_stops
%type <int> arguments
%type <int> connector_symbol
%type <int> assignment_symbol
%type <string> literal
%type <string> cpnt_type
%type <string> is_model
%type <string> clone_arg
%type <string> fsm_decl
%type <ParamType> type
%type <Node*> state_decl
%type <Node*> fsm_items
%type <Node*> component_decl
%type <Node*> container_decl
%type <Node*> assignment_seq_decl
%type <Node*> switch_decl
%type <Node*> pixmap_cache_decl
%type <Node*> poly_decl
%type <Node*> gradient_decl
%type <Node*> path_decl
%type <Node*> path_point_decl
%type <Node*> simple_component_decl
%type <Node*> string_decl
%type < vector<string> > process_list
%type <Node*> full_expression
%type <Node*> expression_term
%type <Node*> cat_expression
%type <Node*> cat_term
%type <Node*> alternative
%type <Node*> start_alternative
%type <Node*> left_side
%type <Node*> right_side
%type <SmalaNative*> lambda
%type <SmalaNative*> start_lambda
%type < vector< pair<ParamType, string> > > parameters
%type < pair<ParamType, string> > parameter

%left QUESTION_MARK COLON
%nonassoc GT GE LT LE EQ NEQ
%left OR AND
%left PLUS MINUS 
%left TIMES DIVIDE
%right NOT


%start program

%%

program: preamble body

preamble:
| preamble use
| preamble import
| preamble native_code { driver.end_debug (); }

use: USE NAME_OR_PATH
{
  driver.add_use ($2);
}

import: IMPORT NAME_OR_PATH
{
  driver.add_import ($2);
}

native_code: native_action | smala_action | rough_code | native_java

native_java: NATIVE_JAVA CODE {
  string str = $2.substr (2, $2.length () - 4);
  driver.add_native_java (str);
}

native_action: NATIVE_ACTION NAME_OR_PATH LP COMPONENT NAME_OR_PATH RP CODE
{
  string str = $7.substr (2, $7.length () - 4);
  driver.add_native_action ($2, $5, str);
}
|
NATIVE_ACTION NAME_OR_PATH LP PROCESS NAME_OR_PATH RP CODE
{
  string str = $7.substr (2, $7.length () - 4);
  driver.add_native_action ($2, $5, str);
}

smala_action: smala_native_start LCB item_list RCB 
{
  Node *node = new Node ();
  node->set_node_type (END_NATIVE);
  driver.add_node (node);
}

smala_native_start: NATIVE_ACTION NAME_OR_PATH LP COMPONENT NAME_OR_PATH COMMA COMPONENT NAME_OR_PATH RP
{
  driver.start_debug ();
  driver.in_preamble ();
  SmalaNative *native = new SmalaNative ($2, $5, $8);
  driver.add_node (native);
}
|
NATIVE_ACTION NAME_OR_PATH LP PROCESS NAME_OR_PATH COMMA PROCESS NAME_OR_PATH RP
{
  driver.start_debug ();
  SmalaNative *native = new SmalaNative ($2, $5, $8);
  driver.add_node (native);
}

rough_code: NATIVE_CODE CODE
{
  string str = $2.substr (2, $2.length () - 4);
  driver.add_native_code (str);
}

body: start_main item action_list { driver.end_debug (); } | define_list

define_list:
| define_list define

define: start_define LCB item_list RCB
{
  driver.end_debug ();
  Node *end = new Node ();
  end->set_node_type (END_DEFINE);
  driver.add_node (end);
  parent_list.pop_back ();
}

action_list:
| action_list action


//------------------------------------------------

start_main: MAIN
{
  driver.end_preamble ();
  driver.start_debug ();
  Node *start = new Node ();
  start->set_node_type (START_MAIN);
  driver.add_node (start);
  driver.set_is_main (true);
}

start_define: DEFINE NAME_OR_PATH LP parameters RP
{
  driver.end_preamble ();
  driver.start_debug ();
  driver.set_is_main (false);
  Node *node = new Node  ("define_start", $2, $4);
  node->set_node_type (START_DEFINE);
  driver.add_define_node (node);
  driver.add_node (node);

  node = new Node ("Component", "this");
  node->set_node_type (THIS);
  parent_list.push_back (node);
  driver.add_node (node);
}

parameters: { vector< pair<ParamType, string> > params; $$ = params; }
| parameters parameter COMMA
{
  $1.push_back ($2);
  $$ = $1;
}
| parameters parameter
{
  $1.push_back ($2);
  $$ = $1;
}

parameter: type NAME_OR_PATH
{
  $$ = make_pair ($1, $2);
}

type: INT_T
{
  $$ = INT;
}
| DOUBLE_T
{
  $$ = DOUBLE;
}
| STRING_T
{
  $$ = STRING;
}
| COMPONENT
{
  $$ = NAME;
}
| PROCESS
{
  $$ = NAME;
}

//------------------------------------------------

items:  { $$ = 0; } | 
LCB item_list RCB 
{
  $$ = 1;
}

item_list:
| item_list item

item: simple_component | connector | binding | assignment | container | alias | set_value | get_value | add_child | load_xml
  | find | native | c_call | action | merge | clone | remove | string_cat | rough_code | macro

macro : NAME_OR_PATH COLON literal
{
  Node *n = new Node ($1, $3);
  driver.add_node (n);
  n->set_node_type (MACRO);
}

literal:
INT
{
  $$ = $1;
}
|
DOUBLE
{
  $$ = $1;
}
|
STRING
{
  $$ = $1;
}

string_cat: STRING_CPNT NAME_OR_PATH SIMPLE_EQ cat_expression
{
  $4->set_name ($2);
}
|
string_decl arguments
{
   ArgNode *n = new ArgNode (END, "");
   driver.add_node (n);
}

string_decl: STRING_CPNT NAME_OR_PATH
{
  Node *n = new Node ("String", $2);
  driver.add_node (n);
  n->set_has_arguments (true);
  n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  $$ = n;
}

cat_expression:
cat_expression PLUS cat_term
{
  OperatorNode *node = new OperatorNode ("TextCatenator", "");
  node->set_node_type (CAT);
  node->set_left ($1);
  node->set_right ($3);
  driver.add_node (node);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  $$ = node; 
}
|
cat_term
{
  $$ = $1;
}

cat_term:
NAME_OR_PATH
{
  Node *node = new Node ("Name", $1);
  node->set_node_type (PATH);
  $$ = node;
}
|
STRING
{
  Node *node = new Node ("String", $1);
  node->set_node_type (LITERAL);
  $$ = node;
}

action: ACTION process_list
{
  $1[0] = std::toupper ($1[0]);
  InstructionNode *node = new InstructionNode ($1);
  for (int i = 0; i < $2.size (); ++i) {
    node->add_cpnt ($2.at (i));
  }
  driver.add_node (node);
}

alias: NAME_OR_PATH AKA NAME_OR_PATH
{
  BinaryInstructionNode *node = new BinaryInstructionNode ($1, $3);
  node->set_node_type (ALIAS);
  driver.add_node (node);
}

merge: MERGE NAME_OR_PATH WITH NAME_OR_PATH
{
  BinaryInstructionNode *node = new BinaryInstructionNode ($2, $4);
  node->set_node_type (MERGE);
  driver.add_node (node);
}

remove: REMOVE NAME_OR_PATH FROM NAME_OR_PATH
{
  BinaryInstructionNode *node = new BinaryInstructionNode ($2, $4);
  node->set_node_type (REMOVE);
  driver.add_node (node);
}

c_call:  start_native_call argument_list RP
{
  ArgNode *n = new ArgNode (END, "");
  driver.add_node (n);
}
| type NAME_OR_PATH SIMPLE_EQ NATIVE_CALL LP NAME_OR_PATH RP
{
  NativeCallNode *n = new NativeCallNode ($1, $2, $6);
  driver.add_node (n);
}

start_native_call: type NAME_OR_PATH SIMPLE_EQ NATIVE_CALL LP NAME_OR_PATH COMMA
{
  NativeCallNode *n = new NativeCallNode ($1, $2, $6);
  n->set_has_arguments (true);
  driver.add_node (n);
}

native: NATIVE NAME_OR_PATH LP NAME_OR_PATH COMMA INT RP
{
  vector< pair<ParamType, string> >d_args;
  d_args.push_back (make_pair (LOCAL_NAME, $4));
  d_args.push_back (make_pair (INT, "0"));
  d_args.push_back (make_pair (INT, $6));
  Node *n = new Node ("NativeAction", $2, d_args);
  n->set_node_type (NATIVE_ACTION_CPNT);
  driver.add_node (n);
  n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
}
| NATIVE NAME_OR_PATH LP NAME_OR_PATH COMMA NAME_OR_PATH COMMA INT RP
{
  vector< pair<ParamType, string> >d_args;
  d_args.push_back (make_pair (LOCAL_NAME, $4));
  d_args.push_back (make_pair (NAME, $6));
  d_args.push_back (make_pair (INT, $8));
  Node *n = new Node ("NativeAction", $2, d_args);
  n->set_node_type (NATIVE_ACTION_CPNT);
  driver.add_node (n);
  n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
}

load_xml: NAME_OR_PATH SIMPLE_EQ LOAD_XML LP STRING RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (STRING, $5));
  Node* n = new Node ("loadFromXML", $1, args);
  n->set_node_type (LOAD_XML);
  driver.add_node (n);
}
|
NAME_OR_PATH SIMPLE_EQ LOAD_XML LP NAME_OR_PATH RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (NAME, $5));
  Node* n = new Node ("loadFromXML", $1, args);
  n->set_node_type (LOAD_XML);
  driver.add_node (n);
}

find: NAME_OR_PATH SIMPLE_EQ FIND LP NAME_OR_PATH RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (NAME, $5));
  Node* n = new Node ("find", $1, args);
  n->set_node_type (FIND);
  driver.add_node (n);
}
| NAME_OR_PATH SIMPLE_EQ FIND LP STRING RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (STRING, $5));
  Node* n = new Node ("find", $1, args);
  n->set_node_type (FIND);
  driver.add_node (n);
}
| NAME_OR_PATH SIMPLE_EQ FIND LP NAME_OR_PATH COMMA STRING RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (NAME, $5));
  args.push_back (make_pair (STRING, $7));
  Node* n = new Node ("find", $1, args);
  n->set_node_type (FIND);
  driver.add_node (n);
}

add_child: NAME_OR_PATH INSERT NAME_OR_PATH
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (NAME, $3));
  Node* n = new Node ("addChild", $1, args);
  n->set_node_type (ADD_CHILD);
  driver.add_node (n);
  n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
}
| NAME_OR_PATH SIMPLE_EQ ADD_CHILD LP NAME_OR_PATH RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (NAME, $5));
  Node* n = new Node ("addChild", $1, args);
  n->set_node_type (ADD_CHILD);
  driver.add_node (n);
  n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
}

clone: start_clone items
{
  Node *n = new Node ();
  n->set_node_type (END_CLONE);
  driver.add_node (n);
  parent_list.pop_back ();
}

start_clone: NAME_OR_PATH SIMPLE_EQ CLONE LP NAME_OR_PATH SIMPLE_EQ clone_arg RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (NAME, $5));
  if ($7.find_first_not_of ("0123456789") == string::npos)
    args.push_back (make_pair (INT, $7));
  else
    args.push_back (make_pair (NAME, $7));
  Node *n = new Node ("clone", $1, args);
  n->set_node_type (CLONE);
  driver.add_node (n);
  n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (n);
}

clone_arg: INT { $$ = $1; } | NAME_OR_PATH { $$ = $1; }

set_value: set_bool | set_int | set_double | set_text | set_ref
get_value: get_bool | get_int | get_double | get_string | get_ref

set_bool: NAME_OR_PATH SIMPLE_EQ TRUE
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (INT, "1"));
  Node *n = new Node ("SetBool", $1, args);
  n->set_node_type (SET_PROPERTY);
  driver.add_node (n);
}
| NAME_OR_PATH SIMPLE_EQ FALSE
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (INT, "0"));
  Node *n = new Node ("SetBool", $1, args);
  n->set_node_type (SET_PROPERTY);
  driver.add_node (n);
}

set_int: NAME_OR_PATH SIMPLE_EQ INT
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (INT, $3));
  Node *n = new Node ("SetInt", $1, args);
  n->set_node_type (SET_PROPERTY);
  driver.add_node (n);
}

set_double: NAME_OR_PATH SIMPLE_EQ DOUBLE
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (INT, $3));
  Node *n = new Node ("SetDouble", $1, args);
  n->set_node_type (SET_PROPERTY);
  driver.add_node (n);
}

set_text: NAME_OR_PATH SIMPLE_EQ STRING
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (STRING, $3));
  Node *n = new Node ("SetText", $1, args);
  n->set_node_type (SET_PROPERTY);
  driver.add_node (n);
}

set_ref: NAME_OR_PATH SIMPLE_EQ NAME_OR_PATH
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (NAME, $3));
  Node *n = new Node ("SetRef", $1, args);
  n->set_node_type (SET_PROPERTY);
  driver.add_node (n);
}

get_int: NAME_OR_PATH SIMPLE_EQ GET_INT LP NAME_OR_PATH RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (INT, $5));
  Node *n = new Node ("GetInt", $1, args);
  n->set_node_type (GET_PROPERTY);
  driver.add_node (n);
}

get_double: NAME_OR_PATH SIMPLE_EQ GET_DOUBLE LP NAME_OR_PATH RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (DOUBLE, $5));
  Node *n = new Node ("GetDouble", $1, args);
  n->set_node_type (GET_PROPERTY);
  driver.add_node (n);
}

get_bool: NAME_OR_PATH SIMPLE_EQ GET_BOOL LP NAME_OR_PATH RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (INT, $5));
  Node *n = new Node ("GetBool", $1, args);
  n->set_node_type (GET_PROPERTY);
  driver.add_node (n);
}

get_string: NAME_OR_PATH SIMPLE_EQ GET_STRING LP NAME_OR_PATH RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (STRING, $5));
  Node *n = new Node ("GetText", $1, args);
  n->set_node_type (GET_PROPERTY);
  driver.add_node (n);
}

get_ref: NAME_OR_PATH SIMPLE_EQ GET_REF LP NAME_OR_PATH RP
{
  vector< pair<ParamType, string> > args;
  args.push_back (make_pair (NAME, $5));
  Node *n = new Node ("GetRef", $1, args);
  n->set_node_type (GET_PROPERTY);
  driver.add_node (n);
}


//------------------------------------------------


simple_component: simple_component_decl arguments
{
  if ($2 == 1) {
    $1->set_has_arguments (true);
    ArgNode *n = new ArgNode (END, "");
    driver.add_node (n);
  }
}

simple_component_decl: cpnt_type NAME_OR_PATH
{
  Node *node = new Node ($1, $2);
  driver.add_node (node);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  $$ = node;
}

cpnt_type:
NAME_OR_PATH { $$ = $1;}
|
ASSIGNMENT_CPNT { $$ = "Assignment"; }
|
CONNECTOR_CPNT { $$ = "Connector"; }
|
BINDING_CPNT { $$ = "Binding"; }

arguments: { $$ = 0; }| LP argument_list RP { $$ = 1; }

argument_list:
| argument_list argument comma
| argument_list argument


argument: argument_expression 

argument_expression:
argument_expression plus argument_term
|
argument_expression minus argument_term
|
argument_expression times argument_term
|
argument_expression divide argument_term
|
argument_term


argument_term:
lp argument_expression rp
|
INT
{
  ArgNode *n = new ArgNode (VALUE, $1);
  driver.add_node (n);
}
|
MINUS INT
{
  ArgNode *n = new ArgNode (VALUE, string("-")+$2);
  driver.add_node (n);
}
|
DOUBLE
{
  ArgNode *n = new ArgNode (VALUE, $1);
  driver.add_node (n);
}
|
MINUS DOUBLE
{
  ArgNode *n = new ArgNode (VALUE, string("-")+$2);
  driver.add_node (n);
}
|
STRING
{
  ArgNode *n = new ArgNode (VALUE, $1);
  driver.add_node (n);
}
|
NAME_OR_PATH
{
  ArgNode *n = new ArgNode (VAR, $1);
  driver.add_node (n);
}
|
NULL
{
  ArgNode *n = new ArgNode (SMALA_NULL, "");
  driver.add_node (n);
}

plus: PLUS
{
  ArgNode *n = new ArgNode (SYMBOL, "+");
  driver.add_node (n);
}
minus: MINUS
{
  ArgNode *n = new ArgNode (SYMBOL, "-");
  driver.add_node (n);
}
times: TIMES
{
  ArgNode *n = new ArgNode (SYMBOL, "*");
  driver.add_node (n);
}
divide: DIVIDE
{
  ArgNode *n = new ArgNode (SYMBOL, "/");
  driver.add_node (n);
}
lp: LP
{
  ArgNode *n = new ArgNode (SYMBOL, "(");
  driver.add_node (n);
}
rp: RP
{
  ArgNode *n = new ArgNode (SYMBOL, ")");
  driver.add_node (n);
}
comma: COMMA
{
  ArgNode *n = new ArgNode (SYMBOL, ", ");
  driver.add_node (n);
}


//------------------------------------------------

connector: full_expression connector_symbol process_list
{
  string type;
  if ($2 == 1)
    type = "Connector";
  else
    type = "PausedConnector";
  for (int i = 0; i < $3.size (); ++i) {
    if ($1->node_type () == ALTERNATIVE) {
      AlternativeNode *tmp = static_cast<AlternativeNode*>($1);
      std::vector<CtrlNode*> ctrl_nodes = tmp->ctrl_nodes ();
      Node *out = new Node ("Name", $3.at (i));
          out->set_node_type (PATH);
      for (int j = 0; j < ctrl_nodes.size (); j++) {
        ctrl_nodes.at (j)->set_djnn_type (type);
        ctrl_nodes.at (j)->set_out (out);
      }
    } else {
      CtrlNode *node = new CtrlNode (type, "");
      node->set_in ($1);
      Node *out = new Node ("Name", $3.at (i));
      out->set_node_type (PATH);
      node->set_out (out);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }
  }
  for (auto n: expression) {
    n->set_in_expression (true);
  }
  expression.clear ();
}

connector_symbol: CONNECTOR { $$ = 1; } | PAUSED_CONNECTOR { $$ = 0; }

binding: NAME_OR_PATH ARROW process_list
{
  for (int i = 0; i < $3.size (); ++i) {
    CtrlNode *node = new CtrlNode ("Binding", "");
    Node *in = new Node ("Name", $1);
    in->set_node_type (PATH);
    node->set_in (in);
    Node *out = new Node ("Name", $3.at (i));
    out->set_node_type (PATH);
    node->set_out (out);
    driver.add_node (node);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }
}
| NAME_OR_PATH ARROW lambda
{
  /* first build the NativeACtion Component */
  vector< pair<ParamType, string> >d_args;
  d_args.push_back (make_pair (LOCAL_NAME, $3->fct ()));
  d_args.push_back (make_pair (INT, $3->data ()));
  d_args.push_back (make_pair (INT, "1"));
  Node *native = new Node ("NativeAction", "", d_args);
  native->set_node_type (NATIVE_ACTION_CPNT);
  driver.add_node (native);
  native->set_parent (parent_list.empty()? nullptr : parent_list.back ());

  /* then the binding */
  CtrlNode *node = new CtrlNode ("Binding", "");
  Node *in = new Node ("Name", $1);
  in->set_node_type (PATH);
  node->set_in (in);
  node->set_out (native);
  driver.add_node (node);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
}

lambda: start_lambda LCB item_list RCB
{
  Node *node = new Node ();
  node->set_node_type (END_NATIVE);
  driver.add_node (node);
  driver.end_preamble ();
  $$ = $1;
}

start_lambda: LP NAME_OR_PATH RP
{
  driver.start_debug ();
  driver.in_preamble ();
  string new_name ("func_" + std::to_string (func_num++));
  SmalaNative *native = new SmalaNative (new_name, "_src_", $2);
  driver.add_node (native);
  $$ = native;
}

assignment: full_expression assignment_symbol process_list is_model
{
  string type = $2 == 1 ? "Assignment" : "PausedAssignment";
  for (int i = 0; i < $3.size (); ++i) {
    if ($1->node_type () == ALTERNATIVE) {
      vector< pair<ParamType, string> > params;
      params.push_back (make_pair (INT, $4));
      AlternativeNode *tmp = dynamic_cast<AlternativeNode*>($1);
      std::vector<CtrlNode*> ctrl_nodes = tmp->ctrl_nodes ();
      Node *out = new Node ("Name", $3.at (i));
      out->set_node_type (PATH);
      for (int j = 0; j < ctrl_nodes.size (); j++) {
        ctrl_nodes.at (j)->set_djnn_type (type);
        ctrl_nodes.at (j)->set_out (out);
        ctrl_nodes.at (j)->add_args (params);
      }
    } else {
      vector< pair<ParamType, string> > params;
      params.push_back (make_pair (INT, $4));
      CtrlNode *node = new CtrlNode (type, "", params);
      node->set_in ($1);
      Node *out = new Node ("Name", $3.at (i));
      out->set_node_type (PATH);
      node->set_out (out);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }
  }
  for (auto n: expression) {
  	n->set_in_expression (true);
  }
  expression.clear ();
}
|
NAME_OR_PATH SIMPLE_EQ full_expression assignment_symbol NAME_OR_PATH is_model
{
  string type = $4 == 1 ? "Assignment" : "PausedAssignment";
  if ($3->node_type () == ALTERNATIVE) {
    cerr << "error: Alternative cannot be named \n";
    driver.set_error ();
  } else {
    vector< pair<ParamType, string> > params;
    params.push_back (make_pair (INT, $6));
    CtrlNode *node = new CtrlNode (type, $1, params);
    node->set_in ($3);
    Node *out = new Node ("Name", $5);
    out->set_node_type (PATH);
    node->set_out (out);
    driver.add_node (node);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  }
  for (auto n: expression) {
  	n->set_in_expression (true);
  }
  expression.clear ();
}

assignment_symbol: ASSIGNMENT { $$ = 1; } | PAUSED_ASSIGNMENT { $$ = 0; }

is_model: { $$ = "0"; } | COLON INT { $$ = $2; }

process_list: NAME_OR_PATH { vector<string> dst; dst.push_back ($1); $$ = dst;}
| process_list COMMA NAME_OR_PATH
{
  $1.push_back ($3);
  $$ = $1;
}


//------------------------------------------------


full_expression :
  full_expression PLUS full_expression { $$ = make_BINARY_OP(driver, "Adder", $1, $3); }
| full_expression MINUS full_expression { $$ = make_BINARY_OP(driver, "Subtractor", $1, $3); }
| full_expression TIMES full_expression { $$ = make_BINARY_OP(driver, "Multiplier", $1, $3); }
| full_expression DIVIDE full_expression { $$ = make_BINARY_OP(driver, "Divider", $1, $3); }
| full_expression OR full_expression { $$ = make_BINARY_OP(driver, "Or", $1, $3); }
| full_expression AND full_expression { $$ = make_BINARY_OP(driver, "And", $1, $3); }
| full_expression LT full_expression { $$ = make_BINARY_OP(driver, "StrictAscendingComparator", $1, $3); }
| full_expression LE full_expression { $$ = make_BINARY_OP(driver, "AscendingComparator", $1, $3); }
| full_expression GT full_expression { $$ = make_BINARY_OP(driver, "StrictAscendingComparator", $3, $1); }
| full_expression GE full_expression { $$ = make_BINARY_OP(driver, "AscendingComparator", $3, $1); }
| full_expression EQ full_expression { $$ = make_BINARY_OP(driver, "EqualityComparator", $1, $3); }
| full_expression NEQ full_expression { $$ = make_BINARY_OP_INVERT(driver, "EqualityComparator", $1, $3); }
| MINUS full_expression %prec NOT { $$ = make_UNARY_OP(driver, "SignInverter", $2); }
| NOT full_expression { $$ = make_UNARY_OP(driver, "Not", $2); }
| alternative { $$ = $1; }
| expression_term { $$ = $1; }
;

expression_term:
  LP full_expression RP   { $$ = $2; }
| INT
  {
    Node *node = new Node ("Number", $1);
    node->set_node_type (LITERAL);
    $$ = node;
  }
| DOUBLE
  {
    Node *node = new Node ("Number", $1);
    node->set_node_type (LITERAL);
    $$ = node;
  }
| TRUE
  {
    Node *node = new Node ("Number", "1");
    node->set_node_type (LITERAL);
    $$ = node;
  }
| FALSE
  {
    Node *node = new Node ("Number", "0");
    node->set_node_type (LITERAL);
    $$ = node;
  }
| NAME_OR_PATH
  {
    Node *node = new Node ("Name", $1);
    node->set_node_type (PATH);
    $$ = node;
  }
| STRING
  {
    Node *node = new Node ("String", $1);
    node->set_node_type (TEXT);
    $$ = node;
  }
;

alternative: start_alternative left_side right_side
{
  AlternativeNode* alt = static_cast<AlternativeNode*> ($1);
  if ($2->node_type () == ALTERNATIVE) {
    AlternativeNode* tmp = static_cast<AlternativeNode*> ($2);
    alt->add_ctrl_nodes (tmp->ctrl_nodes ());
  } else {
    CtrlNode *n = static_cast<CtrlNode*> ($2);
    alt->add_ctrl_node (n);
  }
  if ($3->node_type () == ALTERNATIVE) {
    AlternativeNode* tmp = static_cast<AlternativeNode*> ($3);
    alt->add_ctrl_nodes (tmp->ctrl_nodes ());
  } else {
    CtrlNode *n = static_cast<CtrlNode*> ($3);
    alt->add_ctrl_node (n);
  }

  Node *sw_end = new Node ();
  sw_end->set_node_type (END_CONTAINER);
  driver.add_node (sw_end);
  parent_list.pop_back ();

  $$ = $1;
}

start_alternative: full_expression QUESTION_MARK 
{
  AlternativeNode *alt = new AlternativeNode ();

  for (auto n: expression) {
    n->set_in_expression (true);
  }
  expression.clear ();
  string s = "\"undef\"";
  Node *sw = new Node ("Switch", "");
  sw->set_has_arguments (true);
  sw->set_node_type (CONTAINER);
  driver.add_node (sw);
  sw->set_parent (parent_list.back ());

  ArgNode *node = new ArgNode (VALUE, s);
  driver.add_node (node);
  node = new ArgNode (END, "");
  driver.add_node (node);

  LocalNode *sw_state = new LocalNode (sw, "state");
  sw_state->set_node_type (LOCAL_NODE);

  CtrlNode *sw_conn = new CtrlNode ("Connector", "");
  sw_conn->set_in ($1);
  sw_conn->set_out (sw_state);
  driver.add_node (sw_conn);
  sw_conn->set_parent (parent_list.back ());

  Node *a = new ActivatorNode (sw);
  driver.add_node (a);
  a->set_parent (parent_list.back ());

  parent_list.push_back (sw);

  Node *true_cpnt = new Node ("Component", "true");
  true_cpnt->set_node_type (CONTAINER);
  true_cpnt->set_parent (sw);
  true_cpnt->set_duplicate_warning (false);
  driver.add_node (true_cpnt);
  parent_list.push_back (true_cpnt);

  $$ = alt;
}

left_side: full_expression COLON
{
  if ($1->node_type () != ALTERNATIVE) {
    CtrlNode *true_conn = new CtrlNode ("Connector", "");
    true_conn->set_in ($1);
    driver.add_node (true_conn);
    true_conn->set_parent (parent_list.back());
    $$ = true_conn;
  } else {
    $$ = $1;
  }

  Node *true_end = new Node ();
  true_end->set_node_type (END_CONTAINER);
  driver.add_node (true_end);

  Node* sw = parent_list.back()->parent ();
  Node *false_cpnt = new Node ("Component", "false");
  false_cpnt->set_node_type (CONTAINER);
  false_cpnt->set_parent (sw);
  false_cpnt->set_duplicate_warning (false);
  driver.add_node (false_cpnt);

  parent_list.pop_back();
  parent_list.push_back (false_cpnt);
}

right_side: full_expression
{
  if ($1->node_type () != ALTERNATIVE) {
    CtrlNode *false_conn = new CtrlNode ("Connector", "");
    false_conn->set_in ($1);
    driver.add_node (false_conn);
    false_conn->set_parent (parent_list.back());
    $$ = false_conn;
  } else {
    $$ = $1;
  }

  Node *false_end = new Node ();
  false_end->set_node_type (END_CONTAINER);
  driver.add_node (false_end);
  parent_list.pop_back();
}


//------------------------------------------------

container: generic_container | poly | path | gradient | switch | fsm | pixmap_cache | component 
| add_children_to | assignment_sequence

add_children_to: start_add_children_to LCB item_list RCB
{
  Node *node = new Node ();
  node->set_node_type (END_CONTAINER);
  driver.add_node (node);
  parent_list.pop_back ();
}

start_add_children_to: ADD_CHILDREN_TO NAME_OR_PATH
{
  Node *n = new Node ("addChildrenTo", $2);
  n->set_node_type (ADD_CHILDREN_TO);
  n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (n);
  driver.add_node (n);
}

component: component_decl items {
  Node *node = new Node ();
  node->set_node_type (END_CONTAINER);
  driver.add_node (node);
  parent_list.pop_back ();
}

component_decl: COMPONENT NAME_OR_PATH
{
  Node *node = new Node ("Component", $2);
  node->set_node_type (CONTAINER);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (node);
  driver.add_node (node);
  $$ = node;
}

generic_container: container_decl items {
  Node *node = new Node ();
  node->set_node_type (END_CONTAINER);
  driver.add_node (node);
  parent_list.pop_back ();
}

container_decl: CONTAINER NAME_OR_PATH
{
  Node *node = new Node ($1, $2);
  node->set_node_type (CONTAINER);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (node);
  driver.add_node (node);
  $$ = node;
}

poly: poly_decl points
{
  Node *node = new Node ();
  node->set_node_type (END_CONTAINER);
  driver.add_node (node);
  parent_list.pop_back ();
}

poly_decl: POLY NAME_OR_PATH
{
  Node *node = new Node ($1, $2);
  node->set_node_type (CONTAINER);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (node);
  driver.add_node (node);
  $$ = node;
}

points: { $$ = 0; } | LCB point_list RCB { $$ = 1; }

point_list:
| point_list point

point: point_decl arguments
{
  ArgNode *n = new ArgNode (END, "");
  driver.add_node (n);
}

point_decl: POINT NAME_OR_PATH
{
  Node *node = new Node  ("PolyPoint", $2);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  node->set_has_arguments (true);
  driver.add_node (node);
}

path: path_decl path_points
{
  Node *node = new Node ();
  node->set_node_type (END_CONTAINER);
  driver.add_node (node);
  parent_list.pop_back ();
}

path_decl: PATH NAME_OR_PATH
{
  Node *node = new Node ($1, $2);
  node->set_node_type (CONTAINER);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (node);
  driver.add_node (node);
  $$ = node;
}

path_points: { $$ = 0; } | LCB path_point_list RCB { $$ = 1; }

path_point_list:
| path_point_list path_point

path_point: path_point_decl arguments
{
  if ($2 == 1) {
    $1->set_has_arguments (true);
    ArgNode *n = new ArgNode (END, "");
    driver.add_node (n);
  } else {
    $1->set_has_arguments (false);
  }
}

path_point_decl: PATH_POINT NAME_OR_PATH
{
  Node *node = new Node  ($1, $2);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  driver.add_node (node);
  $$ = node;
}

gradient: gradient_start gradient_stops
{
  Node *node = new Node ();
  node->set_node_type (END_CONTAINER);
  driver.add_node (node);
  parent_list.pop_back ();
}

gradient_start: gradient_decl arguments
{
  ArgNode *n = new ArgNode (END, "");
  driver.add_node (n);
}

gradient_decl: GRADIENT NAME_OR_PATH 
{
  Node *node = new Node ($1, $2);
  node->set_node_type (CONTAINER);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (node);
  driver.add_node (node);
  $$ = node;
}

gradient_stops: { $$ = 0; } | LCB gradient_stop_list RCB { $$ = 1; }

gradient_stop_list:
| gradient_stop_list gradient_stop

gradient_stop: gradient_stop_decl arguments
{
  ArgNode *n = new ArgNode (END, "");
  driver.add_node (n);
}

gradient_stop_decl: GRADIENT_STOP NAME_OR_PATH
{
  Node *node = new Node  ("GradientStop", $2);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  driver.add_node (node);
}

switch: switch_decl items
{
  Node *node = new Node ();
  node->set_node_type (SET_PARENT);
  node->set_name ($1->name ());
  driver.add_node (node);
  parent_list.pop_back ();
}

switch_decl: SWITCH NAME_OR_PATH LP NAME_OR_PATH RP
{
  Node *node = new Node ("Switch", $2);
  node->set_has_arguments (true);
  node->set_node_type (SWITCH);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (node);
  driver.add_node (node);

  ArgNode *n = new ArgNode (VALUE, "\"" + $4 + "\"");
  driver.add_node (n);
  n = new ArgNode (END, "");
  driver.add_node (n);

  $$ = node;
}

assignment_sequence: assignment_seq_decl assignment_items
{
  Node *node = new Node ();
  node->set_node_type (END_CONTAINER);
  driver.add_node (node);
  parent_list.pop_back ();
}

assignment_seq_decl: ASSIGNMENT_SEQUENCE NAME_OR_PATH LP INT RP
{
  Node *node = new Node ("AssignmentSequence", $2);
  node->set_has_arguments (true);
  node->set_node_type (CONTAINER);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (node);
  driver.add_node (node);

  ArgNode *n = new ArgNode (VALUE, $4);
  driver.add_node (n);
  n = new ArgNode (END, "");
  driver.add_node (n);

  $$ = node;
}

assignment_items:  { $$ = 0; } |
LCB assignment_item_list RCB
{
  $$ = 1;
}

assignment_item_list:
| assignment_item_list assignment

fsm: fsm_decl fsm_items
{
  $2->set_name ($1);
}

fsm_decl: FSM NAME_OR_PATH
{
  Node *node = new Node  ("FSM", $2);
  node->set_node_type (FSM);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (node);
  driver.add_node (node);
  $$ = $2;
}

fsm_items: LCB fsm_state_list fsm_transition_list RCB
{
  Node *node = new Node ();
  node->set_node_type (SET_PARENT);
  driver.add_node (node);
  parent_list.pop_back ();
  $$ = node;
}

fsm_state_list:
| fsm_state_list state

state: state_decl items {
  Node *node = new Node ();
  node->set_node_type (END_CONTAINER);
  driver.add_node (node);
  parent_list.pop_back ();
}

state_decl: STATE NAME_OR_PATH
{
  Node *node = new Node  ("FSMState", $2);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  node->set_node_type (CONTAINER);
  parent_list.push_back (node);
  driver.add_node (node);
  $$ = node;
}

fsm_transition_list:
| fsm_transition_list transition

transition: NAME_OR_PATH ARROW NAME_OR_PATH LP NAME_OR_PATH RP
{
  vector< pair<ParamType, string> >args;
  args.push_back (make_pair (NAME, $5));
  CtrlNode *node = new CtrlNode ("FSMTransition", "", args);
  Node *in = new Node ("Name", $1);
  in->set_node_type (PATH);
  node->set_in (in);
  Node *out = new Node ("Name", $3);
  out->set_node_type (PATH);
  node->set_out (out);
  driver.add_node (node);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
}
| NAME_OR_PATH ARROW NAME_OR_PATH LP NAME_OR_PATH COMMA NAME_OR_PATH RP
{
  vector< pair<ParamType, string> >args;
  args.push_back (make_pair (NAME, $5));
  args.push_back (make_pair (NAME, $7));
  CtrlNode *node = new CtrlNode ("FSMTransition", "", args);
 
  Node *in = new Node ("Name", $1);
  in->set_node_type (PATH);
  node->set_in (in);
  Node *out = new Node ("Name", $3);
  out->set_node_type (PATH);
  node->set_out (out);
 
  driver.add_node (node);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
}
| LCB process_list RCB ARROW NAME_OR_PATH LP NAME_OR_PATH RP
{
  for (auto s: $2) {
    vector< pair<ParamType, string> >args;
    args.push_back (make_pair (NAME, $7));
    CtrlNode *node = new CtrlNode ("FSMTransition", "", args);
    Node *in = new Node ("Name", s);
    in->set_node_type (PATH);
    node->set_in (in);
    Node *out = new Node ("Name", $5);
    out->set_node_type (PATH);
    node->set_out (out);
    driver.add_node (node);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  }
}
| LCB process_list RCB ARROW NAME_OR_PATH LP NAME_OR_PATH COMMA NAME_OR_PATH RP
{
  for (auto s: $2) {
    vector< pair<ParamType, string> >args;
    args.push_back (make_pair (NAME, $7));
    args.push_back (make_pair (NAME, $9));
    CtrlNode *node = new CtrlNode ("FSMTransition", "", args);

    Node *in = new Node ("Name", s);
    in->set_node_type (PATH);
    node->set_in (in);
    Node *out = new Node ("Name", $5);
    out->set_node_type (PATH);
    node->set_out (out);

    driver.add_node (node);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  }
}

pixmap_cache: pixmap_cache_decl items
{
  Node *node = new Node ();
  node->set_node_type (END_CONTAINER);
  driver.add_node (node);
  parent_list.pop_back ();
}

pixmap_cache_decl: PIXMAP_CACHE NAME_OR_PATH LP DOUBLE COMMA DOUBLE RP
{
  Node *node = new Node ("PixmapCache", $2);
  node->set_has_arguments (true);
  node->set_node_type (CONTAINER);
  node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
  parent_list.push_back (node);
  driver.add_node (node);

  ArgNode *n = new ArgNode (VALUE, $4);
  driver.add_node (n);
  n = new ArgNode (VALUE, ",");
  driver.add_node (n);
  n = new ArgNode (VALUE, $6);
  driver.add_node (n);
  n = new ArgNode (END, "");
  driver.add_node (n);

  $$ = node;
}
;

%%


void Smala::Parser::error (const location &loc , const string &message) {
  smala::ErrorLocation *mloc = driver.location () ;
  int state = yystack_[0].state ;

  cerr << mloc->file () << ":" << mloc->line () << ": error: ";

#define ERR(line, estate, msg) \
    else if(state==estate) { \
       cerr << msg << endl;\
  }
  
  if(0) ; 
  //ERR(1, "Component", "\"Component\" keyword after a _main_ or _define_")
  #include "errors.h"
  else
    cerr << "syntax error" << " (" << mloc->line () << "," << yystack_[0].state << "," << scanner.YYText() <<  ") " << message << endl;

  driver.set_error ();
}






