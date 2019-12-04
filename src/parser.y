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
//%define api.parser.class { Parser }

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
  #include "dash_array_node.h"
  #include "activator_node.h"
  #include "operator_node.h"
  #include "instruction_node.h"
  #include "binary_instruction_node.h"
  #include "ccall_node.h"
  #include "function_node.h"
  #include "smala_native.h"
  #include "ctrl_node.h"
  #include "local_node.h"
  #include "term_node.h"
  #include "newvar_node.h"
  #include "native_action_node.h"
  #include "native_code_node.h"
  #include "native_expression_node.h"
  #include "for_node.h"
  #include "range_node.h"

  using namespace std;

  namespace Smala {
    class Scanner;
    class Driver;
  }

  typedef pair<Smala::ParamType, string> parameter_t;
  typedef vector<parameter_t> parameters_t;

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
  enum cast
  {
    NO_CAST,
    STRING_CAST,
    DOUBLE_CAST,
    PROCESS_CAST
  };

  enum constant_type
  {
    VALUE,
    ID,
    UNKNOWN
  };

  vector<Node*> parent_list;
  vector<Node*> expression;
  vector<Node*> comp_expression;
  vector<TermNode*> arg_expression;
  vector<int> int_array;
  Node* cur_node;
  bool m_in_arguments = false;
  bool m_in_for = false;
  bool m_in_func = false;
  bool has_argument = false;
  int func_num = 0;
  int loc_node_num = 0;
  cast m_cast = NO_CAST;


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

//%expect 15
%lex-param { Smala::Scanner &scanner }
%lex-param { Smala::Driver &driver }
%parse-param { Smala::Scanner &scanner }
%parse-param { Smala::Driver &driver }

%locations
%define parse.trace
%define parse.error verbose
%define api.token.prefix {TOKEN_}

%token DOLLAR "$"
%token ARROW "->"
%token NOT_ARROW "!->"
%token NOT_ARROW_NOT "!->!"
%token ARROW_NOT "->!"
%token CONNECTOR "=>"
%token ASSGNT_CONN "=:>"
%token PAUSED_CONNECTOR "::>"
%token ASSIGNMENT "=:"
%token PAUSED_ASSIGNMENT "::"
%token PROCESS "Process"
%token <string> STRING_CAST "toString"
%token PROCESS_CAST "&"
%token FSM "FSM"
%token STATE "State"
%token SEMICOLON ";"
%token MINUS "-"
%token PLUS "+"
%token INCR "++"
%token DECR "--"
%token TIMES "*"
%token DIVIDE "/"
%token COLON ":"
%token QUESTION_MARK "?"
%token AND "&&"
%token OR "||"
%token LT "<"
%token LE "<="
%token GT ">"
%token GGT ">>"
%token GE ">="
%token EQ "=="
%token SIMPLE_EQ "="
%token NEQ "!="
%token NOT"!"
%token LCB "{"
%token RCB "}"
%token LB "["
%token RB "]"
%token LP "("
%token RP ")"
%token COMMA ","
%token IF "if"
%token THEN "then"
%token ELSE "else"
%token FOR "for"
%token WHILE "while"
%token PRINT "print"
%token INSERT "insert"
%token INT_T "int"
%token DOUBLE_T "double"
%token STRING_T "string"
%token USE "use"
%token NULL "null"
%token ADD_CHILD "addChild"
%token ADD_CHILDREN_TO "addChildrenTo"
%token CLONE "Clone"
%token MERGE "merge"
%token REMOVE "remove"
%token MOVE "move"
%token WITH "with"
%token FROM "from"
%token NATIVE_CALL "CCal|NativeCall"
%token REPEAT "repeat"
%token SET_INT "setInt"
%token SET_DOUBLE "setDouble"
%token SET_BOOL "setBool"
%token DOUBLE_TO_STRING "doubleToString"
%token INT_TO_STRING "intToString"
%token SET_STRING "setString"
%token SET_REF "setRef"
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

%type <bool> bracket
%type <int> arguments
%type <bool> connector_symbol
%type <int> assignment_symbol
%type <int> argument_list
%type <string> function_call
%type <string> number
%type <string> binding_src
%type <bool> is_model
%type <ParamType> type
%type <string> fsm_decl
%type <string> dash_array_decl
%type <Node*> state_decl
%type <Node*> simple_process_decl
%type <InstructionNode*> start_action
%type < vector<string> > process_list
%type <FunctionNode*> start_function
%type <Node*> imperative_assignment
%type <Node*> start_if
%type <Node*> start_eq
%type <ForNode*> start_for
%type <SmalaNative*> lambda
%type <SmalaNative*> start_lambda
%type < parameters_t > parameters
%type < parameter_t > parameter
%type < pair <std::string, std::string> > binding_type
%type <TermNode*> primary_expression

/*
%left QUESTION_MARK COLON
%nonassoc GT GE LT LE EQ NEQ
%nonassoc   INCR DECR
%left OR AND
%left PLUS MINUS 
%left TIMES DIVIDE
%right NOT
*/

%start program

%%

program
  : preamble body

preamble
  :
  | preamble use
  | preamble import
  | preamble native_code { driver.end_debug (); }

use
  : USE NAME_OR_PATH
    {
      driver.add_use ($2);
    }

import
  : IMPORT NAME_OR_PATH
    {
      driver.add_import ($2);
    }

native_code
  : native_action
  | smala_action
  | rough_code

native_action
  : NATIVE_ACTION NAME_OR_PATH LP "Component" NAME_OR_PATH RP CODE
    {
      string str = $7.substr (2, $7.length () - 4);
      driver.add_native_action ($2, $5, str);
    }
  | NATIVE_ACTION NAME_OR_PATH LP PROCESS NAME_OR_PATH RP CODE
    {
      string str = $7.substr (2, $7.length () - 4);
      driver.add_native_action ($2, $5, str);
    }

smala_action
  : smala_native_start LCB statement_list RCB 
    {
      Node *node = new Node ();
      node->set_node_type (END_NATIVE);
      driver.add_node (node);
    }

smala_native_start
  : NATIVE_ACTION NAME_OR_PATH LP "Component" NAME_OR_PATH COMMA "Component" NAME_OR_PATH RP
    {
      driver.start_debug ();
      driver.in_preamble ();
      SmalaNative *native = new SmalaNative ($2, $5, $8);
      driver.add_node (native);
    }
  | NATIVE_ACTION NAME_OR_PATH LP PROCESS NAME_OR_PATH COMMA PROCESS NAME_OR_PATH RP
    {
      driver.start_debug ();
      SmalaNative *native = new SmalaNative ($2, $5, $8);
      driver.add_node (native);
    }

rough_code
  : NATIVE_CODE CODE
    {
      string str = $2.substr (2, $2.length () - 4);
      driver.add_native_code (str);
    }

body
  : start_main statement_list { driver.end_debug (); }
  | define_list

define_list
  : define
  | define_list define

define
  : start_define LCB statement_list RCB
    {
      driver.end_debug ();
      Node *end = new Node ();
      end->set_node_type (END_DEFINE);
      driver.add_node (end);
      parent_list.pop_back ();
    }

//------------------------------------------------

start_main
  : MAIN
    {
      driver.end_preamble ();
      driver.start_debug ();
      Node *start = new Node ();
      start->set_node_type (START_MAIN);
      driver.add_node (start);
      driver.set_is_main (true);
    }

start_define
  : DEFINE NAME_OR_PATH LP parameters RP
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

parameters
  : { vector< pair<ParamType, string> > params; $$ = params; }
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

parameter
  : type NAME_OR_PATH { $$ = make_pair ($1, $2); }

//------------------------------------------------

statement_list
  : statement
  | statement_list statement

statement
  : new_component
  | expression { if (driver.debug()) driver.new_line(); }
  | tree_action
  | imperative_statement { if (driver.debug()) driver.new_line(); }

new_component
  : simple_process
  | range
  | dash_array
  | fsm
  | native

expression
  : connector
  | assignment
  | binding

tree_action
  : add_children_to
  | add_child { if (driver.debug()) driver.new_line(); }
  | alias { if (driver.debug()) driver.new_line(); }
  | action { if (driver.debug()) driver.new_line(); }
  | merge { if (driver.debug()) driver.new_line(); }
  | remove { if (driver.debug()) driver.new_line(); }
  | move { if (driver.debug()) driver.new_line(); }

imperative_statement
  : if_statement
  | for
  | while_loop
  | print
  | c_call
  | get_string
  | imperative_assignment 
    {
      Node *n = new Node ();
      n->set_node_type (END_ASSIGNMENT);
      driver.add_node (n);
    }

imperative_assignment
  : assignment_expression 
    { 
        for (auto n: comp_expression) {
          if (n->node_type() == TERM_NODE) {
            if (((TermNode*)n)->arg_type () == VAR && !((TermNode*)n)->is_in_func ()) ((TermNode*)n)->set_arg_type (CAST_DOUBLE);
          }
        }

      for (auto n: comp_expression) {
        driver.add_node (n);
      }
      comp_expression.clear ();
      $$ = nullptr; 
    }
  |  start_eq assignment_expression
    {
        for (auto n: arg_expression) {
          if (n->arg_type () == VAR && !n->is_in_func ()) n->set_arg_type (CAST_DOUBLE);
        }

      Node *n = new Node ();
      n->set_node_type (END_SET_PROPERTY);
      driver.add_node (n);
      arg_expression.clear ();
      m_in_arguments = false;
      $$ = n;
    }

start_eq
  : type NAME_OR_PATH SIMPLE_EQ
    {
      NewVarNode *n = new NewVarNode ($1, $2);
      driver.add_node (n);
      m_in_arguments = true;
      $$ = n;
    }
  | NAME_OR_PATH SIMPLE_EQ
    {
      Node *n = new Node ("set", $1);
      n->set_node_type (SET_PROPERTY);
      driver.add_node (n);
      m_in_arguments = true;
      $$ = n;
    }

type
  : INT_T { $$ = INT;}
  | DOUBLE_T { $$ = DOUBLE; }
  | STRING_T { $$ = STRING; }
  | PROCESS { $$ = PROCESS; }

print
  : PRINT LP assignment_expression RP
    {
      Node *n = new Node ();
      n->set_node_type (PRINT);
      n->set_expression (comp_expression);
      comp_expression.clear ();
      driver.add_node (n);
    }

while_loop
  : start_while LCB statement_list RCB
    {
      Node *n = new Node ();
      n->set_node_type (END_BLOCK);
      driver.add_node (n);
    }

start_while
  : WHILE LP assignment_expression RP
    {
      Node *n = new Node ();
      n->set_node_type (WHILE);
      n->set_expression (comp_expression);
      comp_expression.clear ();
      driver.add_node (n);
    }

for
  : for_loop lcb statement_list rcb

for_loop
  : start_for lp imperative_assignment semicolon assignment_expression semicolon imperative_assignment rp
    {
      Node* n = new Node();
      n->set_node_type (END_LOOP);
      m_in_for = false;
    }
start_for
  : FOR
    {
      ForNode *n = new ForNode ();
      driver.add_node (n);
      $$ = n;
      m_in_for = true;
    }

if_statement
  : if_exp statement_list end_if_statement %prec IF
  | if_exp statement_list end_if_statement start_else statement_list end_if_statement
  | if_exp statement_list end_if_statement else if_statement

if_exp
  : start_if assignment_expression end_if_exp
    {
      $1->set_expression (comp_expression);
      comp_expression.clear ();
    }

start_if
  : IF LP
    {
      Node *n = new Node ();
      n->set_node_type (START_IF);
      driver.add_node (n);
      $$ = n;
    }

start_else
  : ELSE LCB
    {
      Node *n = new Node ();
      n->set_node_type (START_ELSE);
      driver.add_node (n);
    }

else
  : ELSE
    {
      Node *n = new Node ();
      n->set_node_type (START_ELSEIF);
      driver.add_node (n);
    }

end_if_exp
  : RP LCB
    {
      Node *n = new Node ();
      n->set_node_type (END_IF_EXPRESSION);
      driver.add_node (n);
    }

end_if_statement
  : RCB
    {
      Node *n = new Node ();
      n->set_node_type (END_BLOCK);
      driver.add_node (n); 
    }

move
  : MOVE NAME_OR_PATH LT NAME_OR_PATH
    {
      BinaryInstructionNode *node = new BinaryInstructionNode ($2, $4);
      node->set_node_type (MOVE_BEFORE);
      driver.add_node (node);
    }
  | MOVE NAME_OR_PATH GT NAME_OR_PATH
    {
      BinaryInstructionNode *node = new BinaryInstructionNode ($2, $4);
      node->set_node_type (MOVE_AFTER);
      driver.add_node (node);
    }
  | MOVE NAME_OR_PATH GGT
    {
      BinaryInstructionNode *node = new BinaryInstructionNode ($2, "");
      node->set_node_type (MOVE_END);
      driver.add_node (node);
    }
  | MOVE NAME_OR_PATH INSERT
    {
      BinaryInstructionNode *node = new BinaryInstructionNode ($2, "");
      node->set_node_type (MOVE_FIRST);
      driver.add_node (node);
    }


action
  : ACTION process_list
    {
      $1[0] = std::toupper ($1[0]);
      InstructionNode *node = new InstructionNode ($1);
      for (int i = 0; i < $2.size (); ++i) {
        node->add_cpnt ($2.at (i));
      }
      driver.add_node (node);
    }
  | start_action argument_list RP
    {
      TermNode *n = new TermNode (END, "");
      comp_expression.push_back (n);
      $1->set_args (comp_expression);
      comp_expression.clear ();
    }

start_action
  : ACTION NAME_OR_PATH LP
    {
      $1[0] = std::toupper ($1[0]);
      InstructionNode *node = new InstructionNode ($1);
      node->add_cpnt ($2);
      node->set_has_argument (true);
      driver.add_node (node);
      $$ = node;
    }

alias
  : NAME_OR_PATH AKA NAME_OR_PATH
    {
      BinaryInstructionNode *node = new BinaryInstructionNode ($1, $3);
      node->set_node_type (ALIAS);
      driver.add_node (node);
    }

merge
  : MERGE NAME_OR_PATH WITH NAME_OR_PATH
    {
      BinaryInstructionNode *node = new BinaryInstructionNode ($2, $4);
      node->set_node_type (MERGE);
      driver.add_node (node);
    }

remove
  : REMOVE NAME_OR_PATH FROM NAME_OR_PATH
    {
      BinaryInstructionNode *node = new BinaryInstructionNode ($2, $4);
      node->set_node_type (REMOVE);
      driver.add_node (node);
    }

c_call
  :  start_native_call argument_list RP
  | type NAME_OR_PATH SIMPLE_EQ NATIVE_CALL LP NAME_OR_PATH RP


start_native_call
  : type NAME_OR_PATH SIMPLE_EQ NATIVE_CALL LP NAME_OR_PATH COMMA
    {
      std::cerr << "\nYou cannot use CCall or NativeCall anymore, use classic functional notation instead.\nEx: t = my_func (\"foo\")\n";
    }

native
  : NATIVE NAME_OR_PATH LP NAME_OR_PATH COMMA INT RP
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
add_child
  : NAME_OR_PATH INSERT NAME_OR_PATH
    {
      vector< pair<ParamType, string> > args;
      args.push_back (make_pair (NAME, $3));
      Node* n = new Node ("addChild", $1, args);
      n->set_node_type (ADD_CHILD);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }
  | NAME_OR_PATH INSERT CLONE LP NAME_OR_PATH RP
    {
      vector< pair<ParamType, string> > args_2;
      args_2.push_back (make_pair (NAME, $5));
      Node* n2 = new Node ("clone", $1, args_2);
      n2->set_node_type (CLONE);
      driver.add_node (n2);

      vector< pair<ParamType, string> > args;
      args.push_back (make_pair (NAME, $1));
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


get_string
  :  NAME_OR_PATH SIMPLE_EQ DOUBLE_TO_STRING LP NAME_OR_PATH RP
    {
      vector< pair<ParamType, string> > args;
      args.push_back (make_pair (DOUBLE, $5));
      Node *n = new Node ("doubleToString", $1, args);
      n->set_node_type (GET_PROPERTY);
      driver.add_node (n);
    }
  | NAME_OR_PATH SIMPLE_EQ INT_TO_STRING LP NAME_OR_PATH RP
    {
      vector< pair<ParamType, string> > args;
      args.push_back (make_pair (INT, $5));
      Node *n = new Node ("intToString", $1, args);
      n->set_node_type (GET_PROPERTY);
      driver.add_node (n);
    }

//------------------------------------------------

dash_array
  : dash_array_decl LP int_array_decl RP
    {
      DashArrayNode *node = new DashArrayNode ($1, int_array);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }

dash_array_decl
  : DASHARRAY NAME_OR_PATH
    {
      int_array.clear ();
      $$ = $2;
    }

int_array_decl
  :
  | int_array_decl INT COMMA { int_array.push_back (std::stoi ($2)); }
  | int_array_decl INT { int_array.push_back (std::stoi ($2)); }

range
  : range_decl start_statement_list end_statement_list
  | range_decl start_statement_list statement_list end_statement_list
  | range_decl

range_decl
  : NAME_OR_PATH bracket number COMMA number bracket
  {
    RangeNode *node = new RangeNode ($1, $3, $2, $5, !$6);
    driver.add_node (node);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    cur_node = node;
    if (driver.debug()) driver.new_line();
  }

bracket
  : LB { $$ = false; }
  | RB { $$ = true; }

number
  : INT { $$ = $1; }
  | DOUBLE { $$ = $1; }

simple_process
  : simple_process_decl arguments start_statement_list statement_list end_statement_list
  | simple_process_decl arguments start_statement_list end_statement_list
  | simple_process_decl start_statement_list statement_list end_statement_list
  | simple_process_decl start_statement_list end_statement_list
  | simple_process_decl arguments
    {
      if ($2) {
        $1->set_has_arguments (true);
        has_argument = false;
      }
      if (driver.debug()) driver.new_line();
    }
  | simple_process_decl
    {
      m_in_arguments = false;
      if (driver.debug()) driver.new_line(); 
    }

start_statement_list
  : LCB
    {
      m_in_arguments = false;
      if (has_argument) {
        cur_node->set_has_arguments (true);
        has_argument = false;
      }
      cur_node->set_node_type (CONTAINER);
      if (cur_node->djnn_type () == "Switch" || cur_node->djnn_type () == "SwitchList")
        cur_node->set_ignore_parent (true);
      parent_list.push_back (cur_node);
    }
end_statement_list
  : RCB 
    {
      if (parent_list.back ()->djnn_type () == "Switch" || parent_list.back ()->djnn_type () == "SwitchList") {
        Node *node = new Node ();
        node->set_node_type (SET_PARENT);
        node->set_name (parent_list.back()->name ());
        parent_list.pop_back ();
        node->set_parent (parent_list.back ());
        driver.add_node (node);
      } else {
        Node *node = new Node ();
        node->set_node_type (END_CONTAINER);
        driver.add_node (node);
        parent_list.pop_back ();
      }
    }

simple_process_decl
  : NAME_OR_PATH NAME_OR_PATH
    {
      Node *node = new Node ($1, $2);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      m_in_arguments = true;
      arg_expression.clear ();
      cur_node = node;
      $$ = node;
    }

arguments
  : LP argument_list RP 
    { 
      if ($2) {
        has_argument = true;
        TermNode *n = new TermNode (END, "");
        driver.add_node (n);
      } else
        has_argument = false;
      m_in_arguments = false;
      comp_expression.clear ();
      arg_expression.clear ();
      $$ = $2; 
    }

argument_list
  : { $$ = 0;}
  | non_empty_argument_list { $$ = 1; }

non_empty_argument_list
  : argument comma non_empty_argument_list
  | argument


argument
  : assignment_expression 
    {
      if (arg_expression.size () > 1) {
        for (auto n: arg_expression) {
          if (n->arg_type () == VAR) {
            n->set_arg_type (CAST_DOUBLE);
          }
        }
      } else if (arg_expression.size () == 1 && arg_expression.at(0)->arg_type () == VAR) {
        arg_expression.at(0)->set_in_func (true);
      }
      arg_expression.clear (); 
    }

assignment_expression
  : conditional_expression

conditional_expression
  : logical_or_expression
  | logical_or_expression question_mark assignment_expression colon conditional_expression

logical_or_expression
  : logical_and_expression
  | logical_or_expression or logical_and_expression

logical_and_expression
  : equality_expression
  | logical_and_expression and equality_expression

equality_expression
  : relational_expression
  | equality_expression eq relational_expression
  | equality_expression neq relational_expression

relational_expression
  : additive_expression
  | relational_expression lt additive_expression
  | relational_expression gt additive_expression
  | relational_expression le additive_expression
  | relational_expression ge additive_expression

additive_expression
  : multiplicative_expression
  | additive_expression plus multiplicative_expression
  | additive_expression minus multiplicative_expression

multiplicative_expression
  : unary_expression
  | multiplicative_expression times unary_expression
  | multiplicative_expression divide unary_expression

unary_operator
  : plus
  | minus
  | not

unary_expression
  : postfix_expression
  | unary_operator unary_expression
  | NAME_OR_PATH INCR
    {
      Node *n = new Node ("incr", $1);
      n->set_node_type (INCREMENT);
      driver.add_node (n);
    }
  | NAME_OR_PATH DECR
    {
      Node *n = new Node ("incr", $1);
      n->set_node_type (DECREMENT);
      driver.add_node (n);
    }
  
function_call
  : start_function argument_list RP
    {
      $1->set_has_arguments ($2);
      $$ = $1->func_name ();
      TermNode *n = new TermNode (SYMBOL, ")");
      if (m_in_arguments || m_in_for) {
        driver.add_node (n);
        arg_expression.push_back (n);
      }
      else
        comp_expression.push_back (n);
      m_in_func = false;
    }
start_function
  : primary_expression LP
    {
      FunctionNode* n = new FunctionNode ($1->arg_value ());
      TermNode *lp = new TermNode (SYMBOL, "(");
      if (m_in_arguments || m_in_for) {
        driver.add_node (n);
        driver.add_node (lp);
        driver.remove_node ($1);
      }
      else {
        std::vector<Node*>::iterator it = find (comp_expression.begin(), comp_expression.end(), $1);
        if (it != comp_expression.end()) {
          comp_expression.erase (it);
        }
        comp_expression.push_back (n);
        comp_expression.push_back (lp);
      }
      arg_expression.clear ();
      if (n->func_name () != "find")
        m_in_func = true;
      $$ = n;
    }

postfix_expression
  : primary_expression
  | function_call 
  | cast primary_expression { m_cast = NO_CAST; }

primary_expression
  : INT
    {
      TermNode *n = new TermNode (VALUE, $1);
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);

      arg_expression.push_back (n);
      $$ = n; 
    }
  | TRUE
    {
      TermNode *n = new TermNode (VALUE, "1");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
      $$ = n; 
    }
  | FALSE
    {
      TermNode *n = new TermNode (VALUE, "0");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
      $$ = n; 
    }
  | DOUBLE
    {
      TermNode *n = new TermNode (VALUE, $1);
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
      $$ = n; 
    }
  | STRING
    {
      TermNode *n = new TermNode (STRING_VALUE, $1);
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
      $$ = n; 
    }
  | NAME_OR_PATH
  {
    TermNode *n;
    if ($1.substr(0,3) == "DJN")
      n = new TermNode (VALUE, $1);
    else {
      switch (m_cast) {
       case STRING_CAST:
         n = new TermNode (CAST_STRING, $1);
         break;
       case DOUBLE_CAST:
         n = new TermNode (CAST_DOUBLE, $1);
         break;
       case PROCESS_CAST:
         n = new TermNode (CAST_PROCESS, $1);
         break;
       default: {
        if (m_in_func)
          n = new TermNode (CAST_DOUBLE, $1);
        else
          n = new TermNode (VAR, $1);
        }
      }
    }
    if (m_in_arguments || m_in_for)
      driver.add_node (n);
    else
      comp_expression.push_back (n);
    arg_expression.push_back (n);
    $$ = n;
  }
  | NULL
    {
      TermNode *n = new TermNode (SMALA_NULL, "");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
      $$ = n; 
    }
  | lp assignment_expression rp { $$ = nullptr; }

cast
  : STRING_CAST 
      {
        if ($1 == "isString") {
          std::cout << "\nWARNING: isString is deprecated, use toString instead\n\n";
        }
        m_cast = STRING_CAST;
      }
  | PROCESS_CAST { m_cast = PROCESS_CAST; }
  | DOLLAR { m_cast = DOUBLE_CAST; }

plus
  : PLUS
    {
      TermNode *n = new TermNode (SYMBOL, "+");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
minus
  : MINUS
    {
      TermNode *n = new TermNode (SYMBOL, "-");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
times
  : TIMES
    {
      TermNode *n = new TermNode (SYMBOL, "*");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
divide
  : DIVIDE
    {
      TermNode *n = new TermNode (SYMBOL, "/");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
or
  : OR
    {
      TermNode *n = new TermNode (SYMBOL, "||");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
and
  : AND
    {
      TermNode *n = new TermNode (SYMBOL, "&&");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
gt
  : GT
    {
      TermNode *n = new TermNode (SYMBOL, ">");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
ge
  : GE
    {
      TermNode *n = new TermNode (SYMBOL, ">=");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
lt
  : LT
    {
      TermNode *n = new TermNode (SYMBOL, "<");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
le
  : LE
    {
      TermNode *n = new TermNode (SYMBOL, "<=");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
eq
  : EQ
    {
      TermNode *n = new TermNode (SYMBOL, "==");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
neq
  : NEQ
    {
      TermNode *n = new TermNode (SYMBOL, "!=");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
not
  : NOT
    {
      TermNode *n = new TermNode (SYMBOL, "!");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
lp
  : LP
    {
      TermNode *n = new TermNode (SYMBOL, "(");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
    }
rp
  : RP
    {
      TermNode *n = new TermNode (SYMBOL, ")");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
    }
lcb
  : LCB
    {
      TermNode *n = new TermNode (START_LCB_BLOCK, "{");
      driver.add_node (n);
    }
rcb
  : RCB
    {
      TermNode *n = new TermNode (END_LCB_BLOCK, "}");
      driver.add_node (n);
    }
comma
  : COMMA
    {
      TermNode *n = new TermNode (SYMBOL, ", ");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
    }
semicolon
  : SEMICOLON
    {
      TermNode *n = new TermNode (SYMBOL, "; ");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
    }
question_mark
  : QUESTION_MARK
    {
      TermNode *n = new TermNode (SYMBOL, "?");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }
colon
  : COLON
    {
      TermNode *n = new TermNode (SYMBOL, ":");
      if (m_in_arguments || m_in_for)
        driver.add_node (n);
      else
        comp_expression.push_back (n);
      arg_expression.push_back (n);
    }

//------------------------------------------------

connector
  : assignment_expression connector_symbol process_list
    {
      NativeExpressionNode *expr_node = new NativeExpressionNode (comp_expression, $2, true, true);
      expr_node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      for (int i = 0; i < $3.size (); ++i) {
        expr_node->add_output_node ($3.at (i));
      }
      driver.add_native_expression (expr_node);
      driver.add_node (expr_node);
      comp_expression.clear ();
      arg_expression.clear (); 
    }
  | assignment_expression ASSGNT_CONN process_list
    {
      NativeExpressionNode *expr_node = new NativeExpressionNode (comp_expression, false, true, false);
      expr_node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      for (int i = 0; i < $3.size (); ++i) {
        expr_node->add_output_node ($3.at (i));
      }
      driver.add_native_expression (expr_node);
      driver.add_node (expr_node);
      comp_expression.clear ();
      arg_expression.clear (); 
    }

connector_symbol
  : CONNECTOR { $$ = false; }
  | PAUSED_CONNECTOR { $$ = true; }

binding
  : binding_src binding_type process_list
    {
      for (int i = 0; i < $3.size (); ++i) {
        CtrlNode *node = new CtrlNode ("Binding", "", $2.first, $2.second);
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
  | binding_src binding_type lambda
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
      CtrlNode *node = new CtrlNode ("Binding", "", $2.first, $2.second);
      Node *in = new Node ("Name", $1);
      in->set_node_type (PATH);
      node->set_in (in);
      node->set_out (native);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }

binding_src
  : assignment_expression {
      if (comp_expression.size () == 1) {
        TermNode* n = dynamic_cast<TermNode*>(comp_expression.at(0));
        if (n != nullptr)
          $$ = n->arg_value ();
        else
          driver.set_error ();
      } else {
        NativeExpressionNode *expr_node = new NativeExpressionNode (comp_expression, false, true, false);
        expr_node->set_parent (parent_list.empty()? nullptr : parent_list.back ());

        // build a local bool that will serve as an output for a connector
        // and an input for the binding in construction
        std::string loc_name ("loc_bool");
        loc_name.append (to_string(loc_node_num++));
        Node *node = new Node ("Bool", loc_name);
        node->set_has_arguments (true);
        driver.add_node (node);
        node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
        TermNode *n = new TermNode (VALUE, "0");
        driver.add_node (n);
        n = new TermNode (END, "");
        driver.add_node (n);

        expr_node->add_output_node (loc_name);
        driver.add_native_expression (expr_node);
        driver.add_node (expr_node);
        comp_expression.clear ();
        arg_expression.clear ();
        loc_name.append(".true");
        $$ = loc_name;
      }
      comp_expression.clear ();
      arg_expression.clear ();
    }

binding_type
  : ARROW { $$ = make_pair ("true", "true"); }
  | NOT_ARROW { $$ = make_pair ("false", "true"); }
  | NOT_ARROW_NOT { $$ = make_pair ("false", "false"); }
  | ARROW_NOT { $$ = make_pair ("true", "false"); }

lambda
  : start_lambda LCB statement_list RCB
    {
      Node *node = new Node ();
      node->set_node_type (END_NATIVE);
      driver.add_node (node);
      driver.end_preamble ();
      $$ = $1;
    }

start_lambda
  : LP NAME_OR_PATH RP
    {
      driver.start_debug ();
      driver.in_preamble ();
      string new_name ("func_" + std::to_string (func_num++));
      SmalaNative *native = new SmalaNative (new_name, "_src_", $2);
      driver.add_node (native);
      $$ = native;
    }

assignment
  : assignment_expression assignment_symbol process_list is_model
    {
      NativeExpressionNode *expr_node = new NativeExpressionNode (comp_expression, $2, false, $4);
      expr_node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      for (int i = 0; i < $3.size (); ++i) {
        Node *out = new Node ("Name", $3.at (i));
        out->set_node_type (PATH);
        expr_node->add_output_node ($3.at (i));
      }
      driver.add_native_expression (expr_node);
      driver.add_node (expr_node);
      comp_expression.clear ();
      arg_expression.clear (); 
    }


assignment_symbol
  : ASSIGNMENT { $$ = false; }
  | PAUSED_ASSIGNMENT { $$ = true; }

is_model
  : { $$ = false; }
  | COLON INT { if ($2.compare ("0") == 0) $$ = false; else $$ = true; }

process_list
  : NAME_OR_PATH { vector<string> dst; dst.push_back ($1); $$ = dst;}
  | process_list COMMA NAME_OR_PATH
    {
      $1.push_back ($3);
      $$ = $1;
    }



//------------------------------------------------

add_children_to
  : start_add_children_to LCB statement_list RCB
    {
      Node *node = new Node ();
      node->set_node_type (END_CONTAINER);
      driver.add_node (node);
      parent_list.pop_back ();
    }

start_add_children_to
  : ADD_CHILDREN_TO NAME_OR_PATH
    {
      Node *n = new Node ("addChildrenTo", $2);
      n->set_node_type (ADD_CHILDREN_TO);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      parent_list.push_back (n);
      driver.add_node (n);
    }

fsm
  : fsm_decl fsm_items
    { 
      Node *node = new Node ();
      node->set_node_type (SET_PARENT);
      node->set_name (parent_list.back()->name ());
      parent_list.pop_back ();
      node->set_parent (parent_list.back ());
      driver.add_node (node);
    }

fsm_decl
  : FSM NAME_OR_PATH
    {
      Node *node = new Node  ("FSM", $2);
      node->set_node_type (FSM);
      node->set_ignore_parent (true);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      parent_list.push_back (node);
      driver.add_node (node);
      if (driver.debug()) driver.new_line();
      $$ = $2;
    }

fsm_items
  : LCB fsm_state_list fsm_transition_list RCB

fsm_state_list
  : state
  | fsm_state_list state

state
  : state_decl { if (driver.debug()) driver.new_line(); }
  | state_decl start_fsm_state statement_list end_fsm_state
  | state_decl start_fsm_state end_fsm_state

state_decl
  : STATE NAME_OR_PATH
    {
      Node *node = new Node  ("FSMState", $2);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      driver.add_node (node);
      cur_node = node;
      $$ = node;
    }

start_fsm_state
  : LCB
    {
      cur_node->set_node_type (CONTAINER);
      parent_list.push_back (cur_node);
    }
end_fsm_state
  : RCB 
    {
      Node *node = new Node ();
      node->set_node_type (END_CONTAINER);
      driver.add_node (node);
      parent_list.pop_back ();
    }

fsm_transition_list
  :
  | fsm_transition_list transition

transition
  : NAME_OR_PATH ARROW NAME_OR_PATH LP NAME_OR_PATH RP
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
      if (driver.debug()) driver.new_line();
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
      if (driver.debug()) driver.new_line();
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
        if (driver.debug()) driver.new_line();
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
        if (driver.debug()) driver.new_line();
      }
    }

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
