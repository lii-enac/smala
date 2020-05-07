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
  #include <algorithm>
  #include <stdint.h>
  #include "node.h"
  #include "path_node.h"
  #include "set_parent_node.h"
  #include "dash_array_node.h"
  #include "instruction_node.h"
  #include "binary_instruction_node.h"
  #include "smala_native.h"
  #include "ctrl_node.h"
  #include "term_node.h"
  #include "newvar_node.h"
  #include "native_action_node.h"
  #include "native_code_node.h"
  #include "native_expression_node.h"
  #include "native_component_node.h"
  #include "for_node.h"
  #include "range_node.h"
  #include "transition_node.h"
  #include "name_context.h"

  using namespace std;

  namespace Smala {
    class Scanner;
    class Driver;
  }

  typedef pair<Smala::ParamType, string> parameter_t;
  typedef vector<parameter_t> parameters_t;

  extern void lexer_expression_mode_on();
  extern void lexer_expression_mode_off();

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

  enum constant_type
  {
    VALUE,
    ID,
    UNKNOWN
  };

  vector<PathNode*> process_list;
  vector<Node*> parent_list;
  vector<SubPathNode*> subpath;
  vector<Node*> expression;
  vector<TermNode*> comp_expression;
  vector<TermNode*> arg_expression;
  vector<NameContext*> name_context_list;
  vector<int> int_array;
  Node *cur_node, *root = nullptr;
  bool m_in_add_children = false;
  bool m_in_arguments = false;
  bool m_in_for = false;
  bool m_in_imperative = false;
  bool has_argument = false;
  int func_num = 0;
  int loc_node_num = 0;
  int m_in_func = 0;
}


%code
{

  bool exclude_from_no_parent (string type) {
    // true is start with Path but is not equal to Path or is equal to Point
    return ((type.rfind ("Path", 0) == 0 && type != "Path") || type == "Point");
  }

  bool is_switch (string type) {
    // true is start with Switch
    return type.rfind ("Switch", 0) == 0;
  }
  bool is_in_expr () {
    if (name_context_list.empty ())
      return false;
    return name_context_list.back ()->in_expr ();
  }
  void
  add_term_node (Smala::Driver &driver, TermNode* n, bool add_to_arg){
    if (is_in_expr ()) {
      name_context_list.back ()->add_term (n);
    } else if (m_in_arguments || m_in_for || m_in_imperative) {
      driver.add_node (n);
    } else
      comp_expression.push_back (n);
    if (add_to_arg)
      arg_expression.push_back (n);
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

%token KEEP "_keep_"
%token EOL "EOL"
%token <string> BREAK "break|continue|return"
%token ACTIVATOR "|->"
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
%token DOT "."
%token COMMA ","
%token IF "if"
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
%token ADD_CHILDREN_TO "addChildrenTo"
%token TO_STRING "toString"
%token MERGE "merge"
%token REMOVE "remove"
%token MOVE "move"
%token WITH "with"
%token FROM "from"
%token MAIN "_main_"
%token DEFINE "_define_"
%token NATIVE "NativeAction"
%token NATIVE_ACTION "_action_"
%token NATIVE_CODE "_native_code_"
%token <string> CODE "<native code>"
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
%token <string> NAME "name"

%type <bool> keep
%type <cast_type> cast
%type <bool> bracket
%type <int> arguments
%type <int> connector_symbol
%type <int> assignment_symbol
%type <int> argument_list
%type <string> function_call
%type <PathNode*> binding_src
%type < std::vector<SubPathNode*> > name_or_path
%type <bool> is_model
%type <ParamType> type
%type <Node*> fsm_decl
%type <string> subname
%type <string> dash_array_decl
%type <Node*> state_decl
%type <Node*> simple_process_decl
%type <Node*> assignment_sequence
%type <Node*> start_assignment_sequence
%type < vector<PathNode*> > process_list
%type < vector<std::string> > state_list
%type <TermNode*> start_function
%type <Node*> imperative_assignment
%type <Node*> start_if
%type <Node*> start_eq
%type <ForNode*> start_for
%type <NativeComponentNode*> lambda
%type <NativeComponentNode*> start_lambda
%type < parameters_t > parameters
%type < parameter_t > parameter
%type < pair <std::string, std::string> > binding_type
%type <TermNode*> primary_expression
%type < std::vector<TermNode*> >first_bound
%type < std::vector<TermNode*> >second_bound

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
  : %empty
  | preamble use
  | preamble import
  | preamble native_code { driver.end_debug (); }

use
  : USE NAME
    {
      driver.add_use ($2);
    }

import
  : IMPORT name_or_path
    {
      driver.add_import (new PathNode ($2));
    }

native_code
  : native_action
  | smala_action
  | rough_code

native_action
  : NATIVE_ACTION NAME LP "Component" NAME RP CODE
    {
      string str = $7.substr (2, $7.length () - 4);
      driver.add_native_action ($2, $5, str);
    }
  | NATIVE_ACTION NAME LP PROCESS NAME RP CODE
    {
      string str = $7.substr (2, $7.length () - 4);
      driver.add_native_action ($2, $5, str);
    }

smala_action
  : smala_native_start LCB statement_list RCB 
    {
      Node *node = new Node (END_NATIVE);
      driver.add_node (node);
    }

smala_native_start
  : NATIVE_ACTION NAME LP "Component" NAME COMMA "Component" name_or_path RP
    {
      driver.start_debug ();
      driver.in_preamble ();
      SmalaNative *native = new SmalaNative ($2, $5, new PathNode ($8));
      driver.add_node (native);
    }
  | NATIVE_ACTION NAME LP PROCESS NAME COMMA PROCESS name_or_path RP
    {
      driver.start_debug ();
      SmalaNative *native = new SmalaNative ($2, $5, new PathNode ($8));
      driver.add_node (native);
    }

rough_code
  : NATIVE_CODE CODE
    {
      string str = $2.substr (2, $2.length () - 4);
      driver.add_native_code (str);
    }

body
  : start_main statement_list
  {
    Node *end_main = new Node (END_MAIN);
    end_main->set_user_data (root);
    driver.add_node (end_main);
    driver.end_debug ();
  }
  | define_list

define_list
  : define
  | define_list define

define
  : start_define LCB statement_list RCB
    {
      driver.end_debug ();
      Node *end = new Node (END_DEFINE);
      driver.add_node (end);
      parent_list.pop_back ();
    }

//------------------------------------------------

start_main
  : MAIN
    {
      driver.end_preamble ();
      driver.start_debug ();
      Node *start = new Node (START_MAIN);
      driver.add_node (start);
      driver.set_is_main (true);
      root = nullptr;
    }

start_define
  : DEFINE NAME LP parameters RP
    {
      driver.end_preamble ();
      driver.start_debug ();
      driver.set_is_main (false);
      Node *node = new Node  (START_DEFINE, "", $2);
      node->set_args_data ($4);
      driver.add_define_node (node);
      driver.add_node (node);

      Node *e_node = new Node (THIS);
      parent_list.push_back (e_node);
      driver.add_node (e_node);
    }

parameters
  : %empty { vector< pair<ParamType, string> > params; $$ = params; }
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
  : type NAME { $$ = make_pair ($1, $2); }

//------------------------------------------------

statement_list
  : statement
  | statement_list statement

statement
  : new_component
  | expression { if (driver.debug()) driver.new_line(); }
  | tree_action
  | imperative_statement { if (driver.debug()) driver.new_line(); }
  | break_loop

new_component
  : simple_process
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
  | imperative_assignment 
    {
      Node *n = new Node (END_ASSIGNMENT);
      driver.add_node (n);
    }

imperative_assignment
  : step eol { lexer_expression_mode_off (); }
  | function_call eol
    { 
      for (auto n: comp_expression) {
        driver.add_node (n);
      }
      comp_expression.clear ();
      lexer_expression_mode_off ();
      $$ = nullptr; 
    }
  | start_eq assignment_expression eol
    {
      Node *n = new Node (END_SET_PROPERTY);
      driver.add_node (n);
      bool has_op = false;
      for (auto n: arg_expression) {
        if (n->arg_type() == OPERATOR)
          has_op = true;
      }
      for (auto n: arg_expression) {
        if (n->arg_type () == VAR && !has_op && n->path_arg_value()->get_cast () == NO_CAST) {
          n->path_arg_value()->set_cast (BY_VALUE);
        }
      }
      arg_expression.clear ();
      m_in_imperative = false;
      lexer_expression_mode_off ();
      $$ = n;
    }

for_imperative_assignment
  : step
  | function_call
    {
      for (auto n: comp_expression) {
        driver.add_node (n);
      }
      comp_expression.clear ();
    }
  | start_eq assignment_expression
    {
      Node *n = new Node (END_SET_PROPERTY);
      driver.add_node (n);
      bool has_op = false;
      for (auto n: arg_expression) {
        if (n->arg_type() == OPERATOR)
          has_op = true;
      }
      for (auto n: arg_expression) {
        if (n->arg_type () == VAR && !has_op && n->path_arg_value()->get_cast () == NO_CAST) {
          n->path_arg_value()->set_cast (BY_VALUE);
        }
      }
      arg_expression.clear ();
      m_in_imperative = false;
    }

start_eq
  : type keep NAME SIMPLE_EQ
    { 
      if (!m_in_for)
        lexer_expression_mode_on ();
      NewVarNode *n = new NewVarNode ($1, $3, $2);
      driver.add_node (n);
      m_in_imperative = true;
      $$ = n;
    }
  | 
    name_or_path SIMPLE_EQ
    {
      if (!m_in_for)
        lexer_expression_mode_on ();
      Node *n = new Node (SET_PROPERTY, "set", new PathNode ($1));
      driver.add_node (n);
      m_in_imperative = true;
      name_context_list.pop_back ();
      $$ = n;
    }

keep
  : %empty { $$ = 0; }
  | KEEP { $$ = 1; }

break_loop
  : BREAK
    {
      Node *n = new Node (BREAK, $1, "");
      driver.add_node (n);
    }

name_or_path
  : NAME
    {
      NameContext *ctxt = new NameContext ();
      name_context_list.push_back (ctxt);
      ctxt->add_subpath (new SubPathNode ($1, START, NO_CAST));
      $$ = ctxt->path ();
    }
  | cast NAME
    {
      NameContext *ctxt = new NameContext ();
      name_context_list.push_back (ctxt);
      ctxt->add_subpath (new SubPathNode ($2, START, $1));
      $$ = ctxt->path ();
    }
  | name_or_path DOT subname
    {
      NameContext *ctxt = name_context_list.back ();
      ctxt->add_subpath (new SubPathNode ($3, ITEM));
      $$ = ctxt->path ();
    }
  | name_or_path DOT DOLLAR NAME
    {
      NameContext *ctxt = name_context_list.back ();
      ctxt->add_subpath (new SubPathNode ("$" + $4, REF));
      $$ = ctxt->path ();
    }
  | name_or_path DOT start_name_expr assignment_expression end_name_expr
    {
      NameContext *ctxt = name_context_list.back ();
      ctxt->build_and_add_expression ();
      $$ = ctxt->path ();
    }
  | name_or_path start_name_expr assignment_expression end_name_expr
    {
      NameContext *ctxt = name_context_list.back ();
      ctxt->build_and_add_expression ();
      $$ = ctxt->path ();
    }

subname
: ACTION { $$ = $1; }| NAME { $$ = $1; } | FROM { $$ = "from"; }

cast
: PROCESS_CAST { $$ = BY_PROCESS; }
| DOLLAR { $$ = BY_VALUE; }

start_name_expr
  : LB { name_context_list.back ()->set_in_expr (true); }
end_name_expr
  : RB { name_context_list.back ()->set_in_expr (false); }

type
  : INT_T { $$ = INT;}
  | DOUBLE_T { $$ = DOUBLE; }
  | STRING_T { $$ = STRING; }
  | PROCESS { $$ = PROCESS; }

print
  : PRINT LP assignment_expression RP
    {
      Node *n = new Node (PRINT);
      n->set_expr_data (comp_expression);
      comp_expression.clear ();
      arg_expression.clear ();
      driver.add_node (n);
    }

while_loop
  : start_while LCB statement_list RCB
    {
      Node *n = new Node (END_BLOCK);
      driver.add_node (n);
    }

start_while
  : WHILE LP assignment_expression RP
    {
      Node *n = new Node (WHILE);
      n->set_expr_data (comp_expression);
      comp_expression.clear ();
      arg_expression.clear ();
      driver.add_node (n);
      lexer_expression_mode_off ();
    }

for
  : for_loop lcb statement_list rcb

for_loop
  : start_for lp for_imperative_assignment semicolon assignment_expression semicolon for_imperative_assignment rp
    {
      Node* n = new Node (END_LOOP);
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
    $1->set_expr_data (comp_expression);
    comp_expression.clear ();
    arg_expression.clear ();
    lexer_expression_mode_off ();
    }

start_if
  : IF LP
    {
      Node *n = new Node (START_IF);
      driver.add_node (n);
      $$ = n;
    }

start_else
  : ELSE LCB
    {
      driver.add_node (new Node (START_ELSE));
    }

else
  : ELSE
    {
      driver.add_node (new Node (START_ELSEIF));
    }

end_if_exp
  : RP LCB
    {
      driver.add_node (new Node (END_IF_EXPRESSION));
    }

end_if_statement
  : RCB
    {
      driver.add_node (new Node (END_BLOCK));
    }

move
  : MOVE name_or_path LT name_or_path
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (MOVE_BEFORE, new PathNode ($2), new PathNode ($4)));
    }
  | MOVE name_or_path GT name_or_path
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (MOVE_AFTER, new PathNode ($2), new PathNode ($4)));
    }
  | MOVE name_or_path GGT
    {
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (MOVE_END, new PathNode ($2), nullptr));
    }
  | MOVE name_or_path INSERT
    {
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (MOVE_FIRST, new PathNode ($2), nullptr));
    }


action
  : ACTION process_list
    {
      $1[0] = std::toupper ($1[0]);
      InstructionNode *node = new InstructionNode ($1);
      for (int i = 0; i < $2.size (); ++i) {
        node->add_path ($2.at (i));
      }
      driver.add_node (node);
    }

alias
  : name_or_path AKA name_or_path
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (ALIAS, new PathNode ($1), new PathNode ($3)));
    }

merge
  : MERGE name_or_path WITH name_or_path
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (MERGE, new PathNode ($2), new PathNode ($4)));
    }

remove
  : REMOVE name_or_path FROM name_or_path
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (REMOVE, new PathNode ($2), new PathNode ($4)));
    }

native
  : NATIVE NAME LP NAME COMMA INT RP
    {
      NativeComponentNode *n = new NativeComponentNode ($4, nullptr, $6);
      n->set_name ($2);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }
  | NATIVE NAME LP NAME COMMA name_or_path COMMA INT RP
    {
      NativeComponentNode *n = new NativeComponentNode ($4, new PathNode ($6), $8);
      n->set_name ($2);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      name_context_list.pop_back ();
    }

add_child
  : start_add assignment_expression eol
    {
      Node *n = new Node (END_ADD_CHILD);
      driver.add_node (n);
      arg_expression.clear ();
      m_in_arguments = false;
      lexer_expression_mode_off ();
    }
start_add
  : NAME INSERT
    {
      lexer_expression_mode_on ();
      Node* n = new Node (ADD_CHILD, "addChild", $1);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      m_in_arguments = true;
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
  : DASHARRAY NAME
    {
      int_array.clear ();
      $$ = $2;
    }

int_array_decl
  : %empty
  | int_array_decl INT COMMA { int_array.push_back (std::stoi ($2)); }
  | int_array_decl INT { int_array.push_back (std::stoi ($2)); }

range_list
  : range
  | range_list range

range
  : range_decl start_statement_list end_statement_list
  | range_decl start_statement_list statement_list end_statement_list
  | range_decl

range_decl
  : NAME bracket first_bound COMMA second_bound bracket
  {
    RangeNode *node = new RangeNode ($1, $3, $2, $5, !$6);
    driver.add_node (node);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    cur_node = node;
    if (driver.debug()) driver.new_line();
  }

first_bound
  : assignment_expression {
    vector<TermNode*> first (comp_expression);
    comp_expression.clear ();
    $$ = first;
  }

second_bound
  : assignment_expression {
    vector<TermNode*> second (comp_expression);
    comp_expression.clear ();
    $$ = second;
  }

bracket
  : LB { $$ = false; }
  | RB { $$ = true; }

simple_process
  : simple_process_decl arguments opt_eol start_statement_list statement_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode ($1));
    }
  }
  | simple_process_decl arguments opt_eol start_statement_list range_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode ($1));
    }
  }
  | simple_process_decl arguments opt_eol start_statement_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode ($1));
    }
  }
  | simple_process_decl opt_eol start_statement_list statement_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode ($1));
    }
  }
  | simple_process_decl opt_eol start_statement_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode ($1));
    }
  }
  | simple_process_decl arguments opt_eol
    {
      if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
        driver.add_node (new SetParentNode ($1));
      }
      if ($2) {
        $1->set_has_arguments (true);
        has_argument = false;
      }
      if (driver.debug()) driver.new_line();
    }
  | simple_process_decl eol
    {
      lexer_expression_mode_off ();
      if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
        driver.add_node (new SetParentNode ($1));
      }
      m_in_arguments = false;
      if (driver.debug()) driver.new_line(); 
    }
  | ACTIVATOR process_list
  {
    for (int i = 0; i < $2.size (); ++i) {
      Node *node = new Node (SIMPLE, "Activator", "_");
      node->set_has_arguments (true);
      driver.add_node (node);
      if (m_in_add_children)
        node->set_ignore_parent (true);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      TermNode *n = new TermNode (VAR, $2.at(i));
      driver.add_node (n);
      n = new TermNode (END, string (""));
      driver.add_node (n);
      if (m_in_add_children) {
        SetParentNode *sp = new SetParentNode (node);
        driver.add_node (sp);
      }
    }
  }

opt_eol
  : %empty { lexer_expression_mode_off ();}
  | eol { lexer_expression_mode_off ();}

eol
  : EOL
  | eol EOL

start_statement_list
  : LCB
    {
      m_in_arguments = false;
      if (has_argument) {
        cur_node->set_has_arguments (true);
        has_argument = false;
      }
      if (cur_node->node_type () != RANGE)
        cur_node->set_node_type (CONTAINER);
      parent_list.push_back (cur_node);
    }
end_statement_list
  : RCB 
    { 
      driver.add_node (new Node (END_CONTAINER));
      parent_list.pop_back ();
    }

simple_process_decl
  : NAME keep NAME
    {
      lexer_expression_mode_on ();
      Node *node = new Node (SIMPLE, $1, $3);
      node->set_keep_name ($2);
      if (root == nullptr)
        root = node;
      driver.add_node (node);
      if ((m_in_add_children && !exclude_from_no_parent ($1)) || is_switch ($1))
        node->set_ignore_parent (true);
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
      } else {
        has_argument = false;
      }
      m_in_arguments = false;
      comp_expression.clear ();
      arg_expression.clear ();
      $$ = $2; 
    }

argument_list
  : %empty { $$ = 0;}
  | non_empty_argument_list { $$ = 1; }

non_empty_argument_list
  : argument comma non_empty_argument_list
  | argument


argument
  : assignment_expression 
    {
      arg_expression.clear (); 
    }

assignment_expression
  : 
  conditional_expression
  {
    bool has_op = false;
    for (auto n: arg_expression) {
      if (n->arg_type() == OPERATOR)
        has_op = true;
    }
    for (auto n: arg_expression) {
      if (n->arg_type () == VAR && has_op && n->path_arg_value()->get_cast () == NO_CAST) {
        n->path_arg_value()->set_cast (BY_REF);
      } else if (m_in_func > 0 && n->arg_type () == VAR && !has_op && n->path_arg_value()->get_cast () == NO_CAST) {
        n->path_arg_value()->set_cast (BY_VALUE);
      }
    }
    has_op = false;
    for (auto n: comp_expression) {
      if (n->arg_type() == OPERATOR)
        has_op = true;
    }
    for (auto n: comp_expression) {
      if (n->arg_type () == VAR && has_op && n->path_arg_value()->get_cast () == NO_CAST) {
        n->path_arg_value()->set_cast (BY_REF);
      }
    }
  }

conditional_expression
  : logical_or_expression
  | logical_or_expression question_mark assignment_expression colon conditional_expression
  {
    add_term_node (driver, new TermNode (SYMBOL, std::string (")")), false);
  }

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

step
  : name_or_path INCR
    {
      if (!m_in_for && !m_in_arguments)
        lexer_expression_mode_on ();
      name_context_list.pop_back ();
      driver.add_node (new Node (INCREMENT, "incr", new PathNode ($1)));
    }
  | name_or_path DECR
    {
      if (!m_in_for && !m_in_arguments)
        lexer_expression_mode_on ();
      name_context_list.pop_back ();
      driver.add_node (new Node (DECREMENT, "incr", new PathNode ($1)));
    }
  
function_call
  : start_function argument_list RP
    {
      $1->set_has_arguments ($2);
      $$ = $1->str_arg_value ();
      add_term_node (driver, new TermNode (SYMBOL, std::string (")")), false);
      m_in_func--;
    }
  | start_to_string name_or_path RP
  {

    TermNode* n = new TermNode (FUNCTION_CALL, "toString");
    add_term_node (driver, n, false);
    add_term_node (driver, new TermNode (SYMBOL, std::string ("(")), false);
    n = new TermNode (VAR, new PathNode ($2));
    n->path_arg_value()->set_cast (BY_REF);
    add_term_node (driver, n, false);
    add_term_node (driver, new TermNode (SYMBOL, std::string (")")), false);
    arg_expression.clear ();
    $$ = "toString";
  }

start_to_string
  : TO_STRING LP
    {
      if (!m_in_for && !m_in_arguments)
        lexer_expression_mode_on ();
    }

start_function
  : NAME LP
    {
      if (!m_in_for && !m_in_arguments) {
        lexer_expression_mode_on ();
      } 
      TermNode* n = new TermNode (FUNCTION_CALL, $1);
      TermNode *lp = new TermNode (SYMBOL, std::string ("("));
      if (is_in_expr ()) {
        NameContext* name_context = name_context_list.back ();
        name_context->add_term (n);
        name_context->add_term (lp);
      } else if (m_in_arguments || m_in_for || m_in_imperative) {
        driver.add_node (n);
        driver.add_node (lp);
      } else  {
        comp_expression.push_back (n);
        comp_expression.push_back (lp);
      }
      arg_expression.clear ();
      m_in_func++;
      $$ = n;
    }

postfix_expression
  : primary_expression
  | function_call

primary_expression
  : INT
    {
      TermNode *n = new TermNode (VALUE, std::string ($1));
      add_term_node (driver, n, true);
      $$ = n; 
    }
  | TRUE
    {
      TermNode *n = new TermNode (VALUE, std::string ("1"));
      add_term_node (driver, n, true);
      $$ = n; 
    }
  | FALSE
    {
      TermNode *n = new TermNode (VALUE, std::string ("0"));
      add_term_node (driver, n, true);
      $$ = n; 
    }
  | DOUBLE
    {
      TermNode *n = new TermNode (VALUE, std::string ($1));
      add_term_node (driver, n, true);
      $$ = n; 
    }
  | STRING
    {
      TermNode *n = new TermNode (STRING_VALUE, std::string ($1));
      add_term_node (driver, n, true);
      $$ = n; 
    }
  | name_or_path
  {
    name_context_list.pop_back ();
    TermNode *n;
    if (!$1.empty () && $1.at (0)->get_subpath ().substr(0,3) == "DJN") {
      n = new TermNode (VALUE, $1.at (0)->get_subpath ());
    }
    else {
      n = new TermNode (VAR, new PathNode ($1));
    }
    if (is_in_expr ()) {
      if (n->arg_type() <= 3)
        driver.add_node (n);
      else
        name_context_list.back ()->add_term (n);
    } else if (m_in_arguments || m_in_for || m_in_imperative) {
      driver.add_node (n);
    } else
      comp_expression.push_back (n);
    arg_expression.push_back (n);
    $$ = n;
  }
  | NULL
    {
      TermNode *n = new TermNode (SMALA_NULL, std::string (""));
      add_term_node (driver, n, true);
      $$ = n; 
    }
  | lp assignment_expression rp { $$ = nullptr; }

plus
  : PLUS
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("+")), true);
    }
minus
  : MINUS
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("-")), true);
    }
times
  : TIMES
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("*")), true);
    }
divide
  : DIVIDE
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("/")), true);
    }
or
  : OR
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("||")), true);
    }
and
  : AND
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("&&")), true);
    }
gt
  : GT
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string (">")), true);
    }
ge
  : GE
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string (">=")), true);
    }
lt
  : LT
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("<")), true);
    }
le
  : LE
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("<=")), true);
    }
eq
  : EQ
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("==")), true);
    }
neq
  : NEQ
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("!=")), true);
    }
not
  : NOT
    {
      add_term_node (driver, new TermNode (OPERATOR, std::string ("!")), true);
    }
lp
  : LP
    {
      add_term_node (driver, new TermNode (SYMBOL, std::string ("(")), false);
    }
rp
  : RP
    {
      add_term_node (driver, new TermNode (SYMBOL, std::string (")")), false);
    }
lcb
  : LCB
    {
      TermNode *n = new TermNode (START_LCB_BLOCK, std::string ("{"));
      driver.add_node (n);
    }
rcb
  : RCB
    {
      TermNode *n = new TermNode (END_LCB_BLOCK, std::string ("}"));
      driver.add_node (n);
    }
comma
  : COMMA
    {
      add_term_node (driver, new TermNode (SYMBOL, std::string (", ")), false);
    }
semicolon
  : SEMICOLON
    {
      add_term_node (driver, new TermNode (SYMBOL, std::string ("; ")), false);
    }
question_mark
  : QUESTION_MARK
    {
      add_term_node (driver, new TermNode (SYMBOL, std::string ("?")), true);
      add_term_node (driver, new TermNode (FUNCTION_CALL, std::string ("smala_deref (")), true);
    }
colon
  : COLON
    {
      add_term_node (driver, new TermNode (SYMBOL, std::string (")")), true);
      add_term_node (driver, new TermNode (SYMBOL, std::string (":")), true);
      add_term_node (driver, new TermNode (FUNCTION_CALL, std::string ("smala_deref (")), true);
    }

//------------------------------------------------

connector
  : assignment_expression connector_symbol process_list
    {
      NativeExpressionNode *expr_node = new NativeExpressionNode (comp_expression, $2 == 0 ? true:false, true, $2 == 2 ? false:true);
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
  : CONNECTOR { lexer_expression_mode_off (); $$ = 1; }
  | PAUSED_CONNECTOR { lexer_expression_mode_off (); $$ = 0; }
  | ASSGNT_CONN { lexer_expression_mode_off (); $$ = 2; }

binding
  : binding_src binding_type process_list
    {
      for (int i = 0; i < $3.size (); ++i) {
        CtrlNode *node = new CtrlNode ("Binding", "", $2.first, $2.second);
        Node *in = new Node (PATH, "Name", $1);
        node->set_in (in);
        Node *out = new Node (PATH, "Name", $3.at (i));
        node->set_out (out);
        driver.add_node (node);
        node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      }
    }
  | binding_src binding_type lambda
    {
      /* first build the NativeACtion Component */
      // vector< pair<ParamType, string> >d_args;
      // d_args.push_back (make_pair (LOCAL_NAME, $3->function_name ()));
      // d_args.push_back (make_pair (INT, "1"));
      // Node *native = new Node (LAMBDA, "NativeAction", "");
      // native->set_args_data (d_args);
      // native->set_path ($3->path_data ());
      driver.add_node ($3);
      $3->set_parent (parent_list.empty()? nullptr : parent_list.back ());

      /* then the binding */
      CtrlNode *node = new CtrlNode ("Binding", "", $2.first, $2.second);
      Node *in = new Node (PATH, "Name", $1);
      node->set_in (in);
      node->set_out ($3);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }
    | binding_src binding_type assignment_sequence {
      CtrlNode *node = new CtrlNode ("Binding", "", $2.first, $2.second);
      Node *in = new Node (PATH, "Name", $1);
      node->set_in (in);
      PathNode *path = new PathNode();
      path->add_subpath (new SubPathNode ($3->name(), START));
      Node *out = new Node (PATH, "Name", path);
      node->set_out (out);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }

assignment_sequence
  : start_assignment_sequence assignment_list RCB
    { 
      driver.add_node (new Node (END_CONTAINER));
      parent_list.pop_back ();
      $$ = $1;
    }

start_assignment_sequence
  : LCB
    {
      std::string loc_name ("loc_ass_seq_" + std::to_string(loc_node_num++));
      Node *node = new Node (SIMPLE, "AssignmentSequence", loc_name);
      driver.add_node (node);

      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      node->set_has_arguments (true);

      driver.add_node (new TermNode (VALUE, "1"));
      driver.add_node (new TermNode (END, ""));

      node->set_node_type (CONTAINER);
      parent_list.push_back (node);
      cur_node = node;
      $$ = node;
    }

assignment_list
  : assignment
  | assignment_list assignment

binding_src
  : assignment_expression
  {
    if (comp_expression.size () == 1) {
      TermNode* n = comp_expression.at (0);
      if (n->arg_type () == VAR) {
        n->path_arg_value()->set_cast (NO_CAST);
        $$ = n->path_arg_value ();
      } else {
        driver.set_error ("Wrong source specification in binding expression");
      }
    } else {
      NativeExpressionNode *expr_node = new NativeExpressionNode (comp_expression, false, true, false);
      expr_node->set_parent (parent_list.empty()? nullptr : parent_list.back ());

      // build a local bool that will serve as an output for a connector
      // and an input for the binding in construction
      std::string loc_name ("loc_bool");
      loc_name.append (to_string(loc_node_num++));
      std::vector<SubPathNode*> path;
      path.push_back (new SubPathNode (loc_name, START));
      Node *node = new Node (SIMPLE, "Bool", loc_name);
      node->set_has_arguments (true);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      TermNode *n = new TermNode (VALUE, "0");
      driver.add_node (n);
      n = new TermNode (END, "");
      driver.add_node (n);

      expr_node->add_output_node (new PathNode (path));
      driver.add_native_expression (expr_node);
      driver.add_node (expr_node);
      comp_expression.clear ();
      arg_expression.clear ();
      path.push_back (new SubPathNode ("true", ITEM));
      loc_name.append(".true");
      $$ = new PathNode (path);
    }
    comp_expression.clear ();
    arg_expression.clear ();
  }

binding_type
  : ARROW { $$ = make_pair ("true", "true"); lexer_expression_mode_off (); }
  | NOT_ARROW { $$ = make_pair ("false", "true"); lexer_expression_mode_off (); }
  | NOT_ARROW_NOT { $$ = make_pair ("false", "false"); lexer_expression_mode_off (); }
  | ARROW_NOT { $$ = make_pair ("true", "false"); lexer_expression_mode_off (); }

lambda
  : start_lambda LCB statement_list RCB
    {
      driver.add_node (new Node (END_NATIVE));
      driver.end_preamble ();
      $$ = $1;
    }

start_lambda
  : LP name_or_path RP
    {
      name_context_list.pop_back ();
      driver.start_debug ();
      driver.in_preamble ();
      string new_name ("func_" + std::to_string (func_num++));
      SmalaNative *native = new SmalaNative (new_name, "_src_", new PathNode ($2));
      driver.add_node (native);
      $$ = new NativeComponentNode (new_name, new PathNode ($2), "1");
    }

assignment
  : assignment_expression assignment_symbol process_list is_model
    {
      NativeExpressionNode *expr_node = new NativeExpressionNode (comp_expression, $2, false, $4);
      expr_node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      for (int i = 0; i < $3.size (); ++i) {
        Node *out = new Node (PATH, "Name", $3.at (i));
        expr_node->add_output_node ($3.at (i));
      }
      driver.add_native_expression (expr_node);
      driver.add_node (expr_node);
      comp_expression.clear ();
      arg_expression.clear (); 
    }


assignment_symbol
  : ASSIGNMENT { $$ = false; lexer_expression_mode_off (); }
  | PAUSED_ASSIGNMENT { $$ = true; lexer_expression_mode_off (); }

is_model
  : %empty { $$ = false; }
  | COLON INT { if ($2.compare ("0") == 0) $$ = false; else $$ = true; }

process_list
  : name_or_path { name_context_list.pop_back (); process_list.clear (); process_list.push_back (new PathNode ($1)); $$ = process_list;}
  | process_list COMMA name_or_path
    {
      name_context_list.pop_back ();
      process_list.push_back (new PathNode ($3));
      $$ = process_list;
    }



//------------------------------------------------

add_children_to
  : start_add_children_to LCB statement_list RCB
    {
      driver.add_node (new Node (END_CONTAINER));
      parent_list.pop_back ();
      m_in_add_children = false;
    }

start_add_children_to
  : ADD_CHILDREN_TO name_or_path
    {
      name_context_list.pop_back ();
      Node *n = new Node (ADD_CHILDREN_TO, "addChildrenTo", new PathNode ($2));
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      parent_list.push_back (n);
      driver.add_node (n);
      m_in_add_children = true;
    }

fsm
  : fsm_decl fsm_items
    {      
      driver.add_node (new Node (END_CONTAINER));
      SetParentNode *p_node = new SetParentNode ($1);
      driver.add_node (p_node);
      parent_list.pop_back ();
    }

fsm_decl
  : FSM NAME
    {
      Node *node = new Node  (FSM, "FSM", $2);
      node->set_ignore_parent (true);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      parent_list.push_back (node);
      driver.add_node (node);
      if (driver.debug()) driver.new_line();
      $$ = node;
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
  : STATE NAME
    {
      Node *node = new Node  (SIMPLE, "FSMState", $2);
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
      driver.add_node (new Node (END_CONTAINER));
      parent_list.pop_back ();
    }

fsm_transition_list
  : %empty
  | fsm_transition_list transition

transition
  : NAME ARROW NAME LP name_or_path RP
    {
      name_context_list.pop_back ();
      TransitionNode *node = new TransitionNode ("FSMTransition", "", $1, $3, new PathNode ($5), nullptr);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      if (driver.debug()) driver.new_line();
    }
  | NAME ARROW NAME LP name_or_path COMMA name_or_path RP
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      TransitionNode *node = new TransitionNode ("FSMTransition", "", $1, $3, new PathNode ($5), new PathNode ($7));

      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      if (driver.debug()) driver.new_line();
    }
  | LCB state_list RCB ARROW NAME LP name_or_path RP
    {
      name_context_list.pop_back ();
      for (auto s: $2) {
        TransitionNode *node = new TransitionNode ("FSMTransition", "", s, $5, new PathNode ($7), nullptr);
        driver.add_node (node);
        node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
        if (driver.debug()) driver.new_line();
      }
    }
  | LCB state_list RCB ARROW NAME LP name_or_path COMMA name_or_path RP
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      for (auto s: $2) {
        TransitionNode *node = new TransitionNode ("FSMTransition", "", s, $5, new PathNode ($7), new PathNode ($9));
        driver.add_node (node);;

        driver.add_node (node);
        node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
        if (driver.debug()) driver.new_line();
      }
    }

  state_list
  : NAME { vector<std::string> dst; dst.push_back ($1); $$ = dst;}
  | state_list COMMA NAME
    {
      $1.push_back ($3);
      $$ = $1;
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
