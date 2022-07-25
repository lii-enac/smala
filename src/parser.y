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
  #include "causal_dep_node.h"
  #include "path_node.h"
  #include "set_parent_node.h"
  #include "dash_array_node.h"
  #include "instruction_node.h"
  #include "binary_instruction_node.h"
  #include "smala_native.h"
  #include "ctrl_node.h"
  #include "newvar_node.h"
  #include "native_action_node.h"
  #include "native_code_node.h"
  #include "native_expression_node.h"
  #include "native_component_node.h"
  #include "for_node.h"
  #include "range_node.h"
  #include "transition_node.h"
  #include "name_context.h"
  #include "forevery_node.h"
  #include "expr_node.h"
  #include "this_node.h"
  #include "add_children_to_node.h"
  #include "self_assign_node.h"
  #include "template_node.h"

  using namespace std;

  namespace Smala {
    class Scanner;
    class Driver;
  }

  typedef pair<Smala::smala_t, string> parameter_t;
  typedef vector<parameter_t> parameters_t;
  typedef vector<Smala::ExprNode*> expression_t;
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



  vector<PathNode*> name_list;
  vector<PathNode*> process_list;
  vector<Node*> parent_list;
  vector<SubPathNode*> subpath;
  vector<Node*> expression;
  vector<BuildNameContext*> name_context_list;
  std::vector<SymTable*> lambda_sym_table_list;
  std::vector<SymTable*> sym_table_list;
  vector<double> double_array;
  Node *cur_node, *root = nullptr;
  //bool m_in_add_children = false;
  bool m_terminate = false;
  bool m_in_lambda = false;
  int m_in_add_children = 0;
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

  void pop_sym_table () {
    if (m_in_lambda)
      lambda_sym_table_list.pop_back ();
    else
      sym_table_list.pop_back ();
  }

  void push_sym_table () {
    if (m_in_lambda)
      lambda_sym_table_list.push_back (new SymTable (lambda_sym_table_list.empty ()? nullptr : lambda_sym_table_list.back ()));
    else
      sym_table_list.push_back (new SymTable (sym_table_list.empty ()? nullptr : sym_table_list.back ()));
  }
  
  void add_sym (const location& loc, const std::string& key, smala_t type) {
    if (m_in_lambda)
      lambda_sym_table_list.back()->add_sym (loc, key, type);
    else
      sym_table_list.back()->add_sym (loc, key, type);
  }

  bool exists (const std::string &key)
  {
    if (m_in_lambda) {
      return lambda_sym_table_list.back ()->exists (key);
    } else {
      return sym_table_list.back ()->exists (key);
    }
  }

  SmalaType get_type (const std::string &key)
  {
    if (m_in_lambda) {
      return lambda_sym_table_list.back ()->get_type (key);
    } else {
      return sym_table_list.back ()->get_type (key);
    }
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

%token AS "as"
%token KEEP "_keep_"
%token EOL "EOL"
%token <string> BREAK "break|continue|return"
%token ACTIVATOR "|->"
%token DOLLAR "$"
%token ARROW "->"
%token NOT_ARROW "!->"
%token NOT_ARROW_NOT "!->!"
%token ARROW_NOT "->!"
%token CAUSAL_DEP "~>"
%token CONNECTOR "=>"
%token ASSGNT_CONN "=:>"
%token PAUSED_CONNECTOR "::>"
%token LAZY_CONNECTOR "=?>"
%token ASSIGNMENT "=:"
%token PAUSED_ASSIGNMENT "::"
%token LAZY_ASSIGNMENT "=?:"
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
%token MINUS_EQ "-="
%token PLUS_EQ "+="
%token TIMES_EQ "*="
%token DIVIDE_EQ "/="
%token COLON ":"
%token QUESTION_MARK "?"
%token MODULO "%"
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
%token OF "of"
%token ELSE "else"
%token FOR "for"
%token WHILE "while"
%token PRINT "print"
%token INSERT "insert"
%token INT_T "int"
%token DOUBLE_T "double"
%token STRING_T "string"
%token LIST_T "list"
%token USE "use"
%token NULL "null"
%token ADD_CHILDREN_TO "addChildrenTo"
%token MERGE "merge"
%token REMOVE "remove"
%token MOVE "move"
%token WITH "with"
%token FROM "from"
%token VALUEOF "valueof"
%token MAIN "_main_"
%token DEFINE "_define_"
%token NATIVE "NativeAction"
%token NATIVE_ASYNC "NativeAsyncAction"
%token NATIVE_COLLECTION "NativeCollectionAction"
%token NATIVE_ACTION "_action_"
%token NATIVE_CODE "_native_code_"
%token NATIVE_CODE_T "NativeCode"
%token <string> CODE "<native code>"
%token DASHARRAY "DashArray"
%token <string> INT "<int>"
%token <string> DOUBLE "<double>"
%token <string> INT_UNIT "<int_unit>"
%token <string> DOUBLE_UNIT "<double_unit>"
%token <string> STRING "<string>"
%token <string> TRUE "TRUE"
%token <string> FALSE "FALSE"
%token <string> ACTION "<action>"
%token AKA "aka"
%token IMPORT "import"
%token END 0 "end of file"
%token <string> NAME "name"

%type <native_type> start_native
%type <bool> keep
%type <bool> bracket
%type < expression_t > arguments
%type <int> connector_symbol
%type <int> assignment_symbol
%type < expression_t > argument_list
%type <PathNode*> binding_src
%type < std::vector<SubPathNode*> > name_or_path
%type <bool> is_model
%type <smala_t> type
%type <Node*> fsm_decl
%type <string> subname
%type <string> dash_array_decl
%type <Node*> state_decl
%type <Node*> simple_process_decl
%type <Node*> assignment_sequence
%type <Node*> start_assignment_sequence
%type <Node*> start_add_child
%type <AddChildrenToNode*> start_add_children_to
%type <ThisNode*> constructor
%type < vector<PathNode*> > process_list
%type < vector<std::string> > state_list
%type <ExprNode*> start_function
%type <Node*> start_if
%type <Node*> start_eq
%type <ForNode*> start_for
%type <NativeComponentNode*> lambda
%type <NativeComponentNode*> start_lambda
%type < parameters_t > parameters
%type < parameter_t > parameter
%type < pair <std::string, std::string> > binding_type
%type < expression_t > non_empty_argument_list
%type <string> unary_operator
%type <ExprNode*> step
%type <Node*> for_imperative_assignment
%type <string> self_assign
%type <ExprNode*> first_bound
%type <ExprNode*> second_bound
%type <ExprNode*> argument
%type <ExprNode*> assignment_expression
%type <ExprNode*> conditional_expression
%type <ExprNode*> logical_or_expression
%type <ExprNode*> logical_and_expression
%type <ExprNode*> equality_expression
%type <ExprNode*> relational_expression
%type <ExprNode*> additive_expression
%type <ExprNode*> multiplicative_expression
%type <ExprNode*> unary_expression
%type <ExprNode*> postfix_expression
%type <ExprNode*> function_call
%type <ExprNode*> primary_expression

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
  : %empty { push_sym_table (); }
  | preamble use
  | preamble import
  | preamble native_code { driver.end_debug (); }

use
  : USE NAME
    {
      driver.add_use (@$, $2);
    }

import
  : IMPORT name_or_path
    {
      driver.add_import (@$, new PathNode (@$, $2));
    }

native_code
  : native_action
  | smala_action
  | rough_code

native_action
  : NATIVE_ACTION NAME LP "Component" NAME RP CODE
    {
      string str = $7.substr (2, $7.length () - 4);
      driver.add_native_action (@$, $2, $5, str);
    }
  | NATIVE_ACTION NAME LP PROCESS NAME RP CODE
    {
      string str = $7.substr (2, $7.length () - 4);
      driver.add_native_action (@$, $2, $5, str);
    }
  | NATIVE_ACTION NAME LP LIST_T NAME COMMA PROCESS NAME RP CODE
    {
      string str = $10.substr (2, $10.length () - 4);
      driver.add_native_collection_action (@$, $2, $5, $8, str);
    }

smala_action
  : smala_native_start LCB statement_list RCB 
    {
      Node *node = new Node (@$, END_NATIVE);
      driver.add_node (node);
      pop_sym_table ();
    }

smala_native_start
  : NATIVE_ACTION NAME LP "Component" NAME COMMA "Component" name_or_path RP
    {
      driver.start_debug ();
      driver.in_preamble ();
      push_sym_table ();
      add_sym (@$, $5, PROCESS);
      int sz = $8.size();
      std::string user_data_name = sz >= 1 ? $8.at (sz - 1)->get_subpath () : "no_user_data";
      add_sym (@$, user_data_name, PROCESS);
      SmalaNative *native = new SmalaNative (@$, $2, $5, new PathNode (@$, $8));
      driver.add_node (native);
    }
  | NATIVE_ACTION NAME LP PROCESS NAME COMMA PROCESS name_or_path RP
    {
      driver.start_debug ();
      SmalaNative *native = new SmalaNative (@$, $2, $5, new PathNode (@$, $8));
      driver.add_node (native);
      push_sym_table ();
      add_sym (@$, $5, PROCESS);
      int sz = $8.size();
      std::string user_data_name = sz >= 1 ? $8.at (sz - 1)->get_subpath () : "no_user_data";
      add_sym (@$, user_data_name, PROCESS);
    }

rough_code
  : NATIVE_CODE CODE
    {
      string str = $2.substr (2, $2.length () - 4);
      driver.add_native_code (@$, str);
    }

body
  : start_main statement_list
  {
    Node *end_main = new Node (@$, END_MAIN);
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
      Node *end = new Node (@$, END_DEFINE);
      end->set_is_define_or_main ();
      driver.add_node (end);
      parent_list.pop_back ();
    }

//------------------------------------------------

start_main
  : MAIN
    {
      driver.end_preamble ();
      driver.start_debug ();
      Node *start = new Node (@$, START_MAIN);
      driver.add_node (start);
      driver.set_is_main (true);
      root = nullptr;
    }

start_define
  : constructor
  | constructor AS function_call
  {
    $1->set_inherit (true);
    $1->set_super_class ($3);
    lexer_expression_mode_off ();
  }

constructor
  : DEFINE NAME LP parameters RP
    {
      driver.end_preamble ();
      driver.start_debug ();
      driver.set_is_main (false);
      Node *node = new Node  (@$, START_DEFINE, "", $2);
      node->set_args_spec ($4);
      driver.add_define_node (node);
      driver.add_node (node);
      for (auto p: $4) {
        if (p.first == NATIVE_CODE_T) {
          node->set_include_native (true);
          break;
        }
      }
      ThisNode *e_node = new ThisNode (@$, "this");
      parent_list.push_back (e_node);
      driver.add_node (e_node);
      add_sym (@$, "this", PROCESS);
      $$ = e_node;
    }


parameters
  : %empty { vector< pair<SmalaType, string> > params; $$ = params; }
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
  : type NAME { add_sym (@$, $2, $1); $$ = make_pair ($1, $2); }

//------------------------------------------------

statement_list
  : statement
  | statement_list statement

statement
  : new_component
  | expression
  | tree_action
  | imperative_statement
  | break_loop

new_component
  : simple_process
  | dash_array
  | fsm
  | native

// FIXME: this should not be called expression, but rather 'coupling' or something else
expression
  : connector
  | assignment
  | binding
  | causal_dep

tree_action
  : add_children_to
  | add_child
  | alias
  | action
  | merge
  | remove
  | move

imperative_statement
  : if_statement
  | for
  | while_loop
  | print
  | imperative_assignment 

imperative_assignment
  : step eol { driver.add_node ($1); lexer_expression_mode_off (); }
  | function_call eol
    { 
      driver.add_node ($1);
      lexer_expression_mode_off ();
    }
  | start_eq assignment_expression eol
    {
      $1->add_arg ($2);
      driver.add_node ($1);
      m_in_imperative = false;
      lexer_expression_mode_off ();
    }

for_imperative_assignment
  : step { $$ = $1; }
  | function_call
    {
      $$ = $1;
    }
  | start_eq assignment_expression
    {
      $1->add_arg ($2);
      m_in_imperative = false;
      $$ = $1;
    }

start_eq
  : type keep NAME SIMPLE_EQ
    { 
      if (!m_in_for)
        lexer_expression_mode_on ();
      NewVarNode *n = new NewVarNode (@$, $1, $3, $2);
      m_in_imperative = true;
      add_sym (@$, $3, $1);
      $$ = n;
    }
  | name_or_path SIMPLE_EQ
    {
      if (!m_in_for)
        lexer_expression_mode_on ();
      Node *n = new Node (@$, SET_PROPERTY, "set", new PathNode (@1, $1));
      m_in_imperative = true;
      name_context_list.pop_back ();
      if ($1.size () == 1) {
        if (!exists ($1.at(0)->get_subpath ())) {
          add_sym (@$, $1.at(0)->get_subpath (), PROCESS);
        }
      }
      $$ = n;
    }
  | name_or_path self_assign
    {
      if (!m_in_for)
        lexer_expression_mode_on ();
      SelfAssignNode *n = new SelfAssignNode (@$, new PathNode (@1, $1), $2);
      m_in_imperative = true;
      name_context_list.pop_back ();
      if ($1.size () == 1) {
        if (!exists ($1.at(0)->get_subpath ())) {
          add_sym (@$, $1.at(0)->get_subpath (), PROCESS);
        }
      }
      $$ = n;
    }
  
  self_assign
  : PLUS_EQ   { $$ = "+"; }
  | MINUS_EQ  { $$ = "-"; }
  | TIMES_EQ  { $$ = "*"; }
  | DIVIDE_EQ { $$ = "/"; }

keep
  : %empty { $$ = 0; }
  | KEEP { $$ = 1; }

break_loop
  : BREAK
    {
      Node *n = new Node (@$, BREAK, $1, "");
      driver.add_node (n);
    }

name_or_path
  : NAME
    {
      m_terminate = false;
      BuildNameContext *ctxt = new BuildNameContext ();
      name_context_list.push_back (ctxt);
      ctxt->add_subpath (new SubPathNode (@$, $1, START, NO_CAST));
      $$ = ctxt->path ();
    }
  | name_or_path DOT subname
    {
      BuildNameContext *ctxt = name_context_list.back ();
      if (m_terminate) {
        driver.set_error ("Invalid path specification");
      }
      ctxt->add_subpath (new SubPathNode (@$, $3, ITEM));
      $$ = ctxt->path ();
    }
  | name_or_path DOT DOLLAR NAME
    {
      BuildNameContext *ctxt = name_context_list.back ();
      if (m_terminate) {
        driver.set_error ("Invalid path specification");
      }
      ctxt->add_subpath (new SubPathNode (@$, "$" + $4, REF));
      $$ = ctxt->path ();
    }
  | name_or_path DOT start_name_expr assignment_expression end_name_expr
    {
      BuildNameContext *ctxt = name_context_list.back ();
      if (m_terminate) {
        driver.set_error ("Invalid path specification");
      }
      ctxt->build_and_add_expression (@$, $4);
      $$ = ctxt->path ();
    }
  | name_or_path start_name_expr assignment_expression end_name_expr
    {
      BuildNameContext *ctxt = name_context_list.back ();
      if (m_terminate) {
        driver.set_error ("Invalid path specification");
      }
      ctxt->build_and_add_expression (@$, $3);
      $$ = ctxt->path ();
    }
  | name_or_path DOT LCB pname_list RCB
    {
      BuildNameContext *ctxt = name_context_list.back ();
      if (m_terminate) {
        driver.set_error ("Invalid path specification");
      }
      ctxt->add_subpath (new SubPathNode (@$, name_list));
      m_terminate = true;
      $$ = ctxt->path ();
    }
    | name_or_path DOT LCB TIMES RCB
    {
      BuildNameContext *ctxt = name_context_list.back ();
      if (m_terminate) {
        driver.set_error ("Invalid path specification");
      }
      SubPathNode *n = new SubPathNode (@$);
      n->set_path_type (WILD_CARD);
      ctxt->add_subpath (n);
      m_terminate = true;
      $$ = ctxt->path ();
    }

pname_list
  : name_or_path { name_list.clear (); name_list.push_back (new PathNode (@$, $1)); name_context_list.pop_back (); }
  | pname_list COMMA name_or_path { name_list.push_back (new PathNode (@$, $3)); name_context_list.pop_back (); }

subname
: ACTION { $$ = $1; }| NAME { $$ = $1; } | FROM { $$ = "from"; }

start_name_expr
  : LB
end_name_expr
  : RB

type
  : INT_T { $$ = INT;}
  | DOUBLE_T { $$ = DOUBLE; }
  | STRING_T { $$ = STRING; }
  | PROCESS { $$ = PROCESS; }
  | NATIVE_CODE_T { $$ = NATIVE_CODE_T; }

print
  : PRINT LP assignment_expression RP
    {
      Node *n = new Node (@$, PRINT);
      n->add_arg ($3);
      driver.add_node (n);
    }

while_loop
  : start_while LCB statement_list RCB
    {
      Node *n = new Node (@$, END_BLOCK);
      driver.add_node (n);
      pop_sym_table ();
    }

start_while
  : WHILE LP assignment_expression RP
    {
      Node *n = new Node (@$, WHILE);
      n->add_arg ($3);
      driver.add_node (n);
      lexer_expression_mode_off ();
      push_sym_table ();
    }

for
  : for_loop LCB statement_list RCB
  {
    Node *n = new Node (@$, END_BLOCK);
    driver.add_node (n);
    pop_sym_table ();
  }
  | forevery_loop LCB statement_list RCB
  {
    Node *n = new Node (@$, END_BLOCK);
    driver.add_node (n);
    pop_sym_table ();
  }

forevery_loop
  : FOR NAME COLON name_or_path
  {
    ForEveryNode *n = new ForEveryNode (@$, $2, new PathNode (@4, $4));
    driver.add_node (n);
    push_sym_table ();
    add_sym (@$, $2, PROCESS);
  }

for_loop
  : start_for LP for_imperative_assignment SEMICOLON assignment_expression SEMICOLON for_imperative_assignment RP
    {
      $1->set_statements ($3, $5, $7);
      m_in_for = false;
    }

start_for
  : FOR
    {
      push_sym_table ();
      ForNode *n = new ForNode (@$);
      driver.add_node (n);
      $$ = n;
      m_in_for = true;
    }

if_statement
  : if_exp statement_list end_if_statement %prec IF
  | if_exp statement_list end_if_statement start_else statement_list end_if_statement
  | if_exp statement_list end_if_statement else if_statement { driver.add_node (new Node (@$, END_BLOCK)); }

if_exp
  : start_if assignment_expression end_if_exp
    {
    $1->add_arg ($2);
    lexer_expression_mode_off ();
    }

start_if
  : IF LP
    {
      Node *n = new Node (@$, START_IF);
      driver.add_node (n);
      $$ = n;
    }

start_else
  : ELSE LCB
    {
      driver.add_node (new Node (@$, START_ELSE));
      push_sym_table ();
    }

else
  : ELSE
    {
      driver.add_node (new Node (@$, START_ELSEIF));
    }

end_if_exp
  : RP LCB
    {
      push_sym_table ();
    }

end_if_statement
  : RCB
    {
      driver.add_node (new Node (@$, END_BLOCK));
      pop_sym_table ();
    }

move
  : MOVE name_or_path LT name_or_path
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (@$, MOVE_BEFORE, new PathNode (@2, $2), new PathNode (@4, $4)));
    }
  | MOVE name_or_path GT name_or_path
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (@$, MOVE_AFTER, new PathNode (@2, $2), new PathNode (@4, $4)));
    }
  | MOVE name_or_path GGT
    {
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (@$, MOVE_END, new PathNode (@2, $2), nullptr));
    }
  | MOVE name_or_path INSERT
    {
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (@$, MOVE_FIRST, new PathNode (@2, $2), nullptr));
    }


action
  : ACTION process_list
    {
      $1[0] = std::toupper ($1[0]);
      InstructionNode *node = new InstructionNode (@$, $1);
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
      driver.add_node (new BinaryInstructionNode (@$, ALIAS, new PathNode (@1, $1), new PathNode (@3, $3)));
      add_sym (@$, $1.at(0)->get_subpath (), PROCESS);
    }

merge
  : MERGE name_or_path WITH name_or_path
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (@$, MERGE, new PathNode (@2, $2), new PathNode (@4, $4)));
    }

remove
  : REMOVE name_or_path FROM name_or_path
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      driver.add_node (new BinaryInstructionNode (@$, REMOVE, new PathNode (@2, $2), new PathNode (@4, $4)));
    }

native
  : start_native NAME LP NAME COMMA INT RP
    {
      NativeComponentNode *n = new NativeComponentNode (@$, $4, nullptr, nullptr, $6, $1);
      n->set_name ($2);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      if ($2 != "_")
        add_sym (@$, $2, PROCESS);
    }
  | start_native NAME LP NAME COMMA name_or_path COMMA INT RP
    {
      NativeComponentNode *n = new NativeComponentNode (@$, $4, nullptr, new PathNode (@6, $6), $8, $1);
      n->set_name ($2);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      name_context_list.pop_back ();
      if ($2 != "_")
        add_sym (@$, $2, PROCESS);
    }
  | NATIVE_COLLECTION NAME LP NAME COMMA name_or_path COMMA name_or_path COMMA INT RP
    {
      NativeComponentNode *n = new NativeComponentNode (@$, $4, new PathNode (@6, $6), new PathNode (@8, $8), $10, COLLECTION_ACTION);
      n->set_name ($2);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      name_context_list.pop_back ();
      if ($2 != "_")
        add_sym (@$, $2, PROCESS);
    }
  | NATIVE_COLLECTION NAME LP NAME COMMA name_or_path COMMA INT RP
    {
      NativeComponentNode *n = new NativeComponentNode (@$, $4, new PathNode (@6, $6), nullptr, $8, COLLECTION_ACTION);
      n->set_name ($2);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      name_context_list.pop_back ();
      if ($2 != "_")
        add_sym (@$, $2, PROCESS);
    }

start_native
  : NATIVE { $$ = SIMPLE_ACTION; }
  | NATIVE_ASYNC { $$ = ASYNC_ACTION; }

add_child
  : start_add_child argument eol
    {
      Node *n = new Node (@$, END_ADD_CHILD);
      driver.add_node (n);
      lexer_expression_mode_off ();
      $1->add_arg ($2);
      if ($1->keep_name ()) {
        n->set_keep_name (true);
        n->set_name ($1->name ());
      }
    }

start_add_child
  : NAME INSERT
    {
      lexer_expression_mode_on ();
      Node* n = new Node (@$, ADD_CHILD, "addChild", $1);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      add_sym (@$, $1, PROCESS);
      $$ = n;
    }
    | VALUEOF NAME INSERT
    {
      lexer_expression_mode_on ();
      Node* n = new Node (@$, ADD_CHILD, "addChild", $2);
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      n->set_keep_name (true);
      $$ = n;
    } 
    |INSERT
    {
      lexer_expression_mode_on ();
      Node* n = new Node (@$, ADD_CHILD, "addChild", "");
      driver.add_node (n);
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      $$ = n;
    }


//------------------------------------------------

dash_array
  : dash_array_decl LP double_array_decl RP
    {
      DashArrayNode *node = new DashArrayNode (@$, $1, double_array);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }

dash_array_decl
  : DASHARRAY NAME
    {
      double_array.clear ();
      $$ = $2;
      if ($2 != "_")
        add_sym (@$, $2, PROCESS);
    }

double_array_decl
  : %empty
  | double_array_decl DOUBLE COMMA { double_array.push_back (std::stof ($2)); }
  | double_array_decl DOUBLE { double_array.push_back (std::stof ($2)); }

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
    RangeNode *node = new RangeNode (@$, $1, $3, $2, $5, !$6);
    driver.add_node (node);
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    cur_node = node;
    if ($1 != "_")
      add_sym (@$, $1, PROCESS);
  }

first_bound
  : assignment_expression {
    $$ = $1;
  }

second_bound
  : assignment_expression {
    $$ = $1;
  }

bracket
  : LB { $$ = false; }
  | RB { $$ = true; }

simple_process
  : simple_process_decl arguments opt_eol start_statement_list statement_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode (@$, $1));
    }
    $1->set_args ($2);
  }
  | simple_process_decl arguments opt_eol start_statement_list range_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode (@$, $1));
    }
    $1->set_args ($2);
  }
  | simple_process_decl arguments opt_eol start_statement_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode (@$, $1));
    }
    $1->set_args ($2);
  }
  | simple_process_decl opt_eol start_statement_list statement_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode (@$, $1));
    }
  }
  | simple_process_decl opt_eol start_statement_list end_statement_list
  {
    if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
      driver.add_node (new SetParentNode (@$, $1));
    }
  }
  | simple_process_decl arguments opt_eol
    {
      if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
        driver.add_node (new SetParentNode (@$, $1));
      }
      $1->set_args ($2);
    }
  | simple_process_decl AKA name_or_path opt_eol
    {
      if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
        driver.add_node (new SetParentNode (@$, $1));
      }
      $1->set_path(new PathNode (@3, $3));
    }
  | simple_process_decl eol
    {
      lexer_expression_mode_off ();
      if ((m_in_add_children && !exclude_from_no_parent ($1->djnn_type ())) || is_switch ($1->djnn_type ())) {
        driver.add_node (new SetParentNode (@$, $1));
      }
      m_in_arguments = false;
    }
  | ACTIVATOR process_list
  {
    for (int i = 0; i < $2.size (); ++i) {
      auto &loc = $2.at(i)->get_location ();
      Node *node = new Node (loc, SIMPLE, "Activator", "_");
      PathExprNode *arg = new PathExprNode ($2.at(i)->get_location (), $2.at(i), PROCESS);
      UnaryExprNode *un = new UnaryExprNode ($2.at(i)->get_location (), "&", arg);
      node->add_arg (un);
      driver.add_node (node);
      if (m_in_add_children)
        node->set_ignore_parent (true);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      if (m_in_add_children) {
        SetParentNode *sp = new SetParentNode (loc, node);
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
      if (cur_node->node_type () != RANGE)
        cur_node->set_node_type (CONTAINER);
      parent_list.push_back (cur_node);
      push_sym_table ();
    }
end_statement_list
  : RCB 
    { 
      driver.add_node (new Node (@$, END_CONTAINER));
      parent_list.pop_back ();
      pop_sym_table ();
    }

simple_process_decl
  : NAME keep NAME
    {
      lexer_expression_mode_on ();
      Node *node = new Node (@$, SIMPLE, $1, $3);
      node->set_keep_name ($2);
      if (root == nullptr) {
        root = node;
        node->set_is_define_or_main ();
      }
      driver.add_node (node);
      if ((m_in_add_children && !exclude_from_no_parent ($1)) || is_switch ($1))
        node->set_ignore_parent (true);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      cur_node = node;
      if ($3.compare("_") != 0)
        add_sym (@$, $3, PROCESS);
      $$ = node;
    }
  | NAME OF NAME keep NAME
    {
      lexer_expression_mode_on ();
      auto *node = new TemplatePropertyNode (@$, SIMPLE, $1, $5, $3);
      node->set_keep_name ($4);
      if (root == nullptr) {
        root = node;
        node->set_is_define_or_main ();
      }
      driver.add_node (node);
      if ((m_in_add_children && !exclude_from_no_parent ($1)) || is_switch ($1))
        node->set_ignore_parent (true);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      cur_node = node;
      if ($5.compare("_") != 0)
        add_sym (@$, $5, PROCESS);
      $$ = node;
    }

arguments
  : LP argument_list RP 
    { 
      $$ = $2; 
    }

argument_list
  : %empty { std::vector <ExprNode*> args; $$ = args; }
  | non_empty_argument_list  { std::reverse($1.begin(), $1.end()); $$ = $1; }

non_empty_argument_list
  : argument COMMA non_empty_argument_list
  {
    $3.push_back ($1);
    $$ = $3;
  }
  | argument {
    std::vector <ExprNode*> args;
    args.push_back ($1);
    $$ = args;
  }


argument
  : assignment_expression 
    {
      if ($1->get_expr_type () == PROCESS) {
        UnaryExprNode *n = new UnaryExprNode (@$, "&", $1);
        $$ = n;
      } else {
        $$ = $1;
      }
    }

// FIXME: this should be called expression
assignment_expression
  : 
  conditional_expression
  {
    $$ = $1;
  }

conditional_expression
  : logical_or_expression { $$ = $1; }
  | logical_or_expression QUESTION_MARK assignment_expression COLON conditional_expression
  {
    TernaryExprNode *n = new TernaryExprNode (@$, $1, $3, $5);
    $$ = n;
  }

logical_or_expression
  : logical_and_expression { $$ = $1; }
  | logical_or_expression OR logical_and_expression 
  { 
    BinaryExprNode* n = new BinaryExprNode (@$, "||", $1, $3);
    $$ = n;
  }

logical_and_expression
  : equality_expression { $$ = $1; }
  | logical_and_expression AND equality_expression { 
    BinaryExprNode* n = new BinaryExprNode (@$, "&&", $1, $3);
    $$ = n;
  }

equality_expression
  : relational_expression { $$ = $1; }
  | equality_expression EQ relational_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, "==", $1, $3);
    $$ = n;
  }
  | equality_expression NEQ relational_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, "!=", $1, $3);
    $$ = n;
  }

relational_expression
  : additive_expression { $$ = $1; }
  | relational_expression LT additive_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, "<", $1, $3);
    $$ = n;
  }
  | relational_expression GT additive_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, ">", $1, $3);
    $$ = n;
  }
  | relational_expression LE additive_expression 
  {
    BinaryExprNode* n = new BinaryExprNode (@$, "<=", $1, $3);
    $$ = n;
  }
  | relational_expression GE  additive_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, ">=", $1, $3);
    $$ = n;
  }

additive_expression
  : multiplicative_expression { $$ = $1; }
  | additive_expression PLUS multiplicative_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, "+", $1, $3);
    $$ = n;
  }
  | additive_expression MINUS multiplicative_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, "-", $1, $3);
    $$ = n;
  }

multiplicative_expression
  : unary_expression { $$ = $1; }
  | multiplicative_expression TIMES unary_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, "*", $1, $3);
    $$ = n;
  }
  | multiplicative_expression DIVIDE unary_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, "/", $1, $3);
    $$ = n;
  }
  | multiplicative_expression MODULO unary_expression
  {
    BinaryExprNode* n = new BinaryExprNode (@$, "%", $1, $3);
    $$ = n;
  }

unary_operator
  : PLUS { $$ = "+"; }
  | MINUS { $$ = "-"; }
  | NOT { $$ = "!"; }
  | PROCESS_CAST { $$ = "&"; }
  | DOLLAR { $$ = "$"; }

unary_expression
  : postfix_expression { $$ = $1; }
  | unary_operator unary_expression
  {
    UnaryExprNode* n = new UnaryExprNode (@$, $1, $2);
    $$ = n;
  }

step
  : name_or_path INCR
    {
      if (!m_in_for && !m_in_arguments)
        lexer_expression_mode_on ();
      name_context_list.pop_back ();
      StepExprNode *n = new StepExprNode (@$, new PathNode (@1, $1), true, get_type ($1.at (0)->get_subpath ()));
      $$ = n;
    }
  | name_or_path DECR
    {
      if (!m_in_for && !m_in_arguments)
        lexer_expression_mode_on ();
      name_context_list.pop_back ();
      StepExprNode *n = new StepExprNode (@$, new PathNode (@1, $1), false, get_type ($1.at (0)->get_subpath ()));
      $$ = n;
    }
  
function_call
  : start_function argument_list RP
    {
      if ($1->get_val().compare("toString") == 0) {
        if ($2.size () != 1) {
          driver.set_error ("Wrong number of arguments for toString");
        } else if ($2.at(0)->get_expr_type () != PROCESS) {
          driver.set_error ("Wrong argument type for toString");
        } else {
          if ($2.at (0)->get_expr_node_type () == UNARY_OP) {
            ExprNode* n = ((UnaryExprNode*)$2.at (0))->get_child ();
            n->set_expr_type (CAST_STRING);
            $$ = n;
          } else {
            $2.at(0)->set_expr_type (CAST_STRING);
            m_in_func--;
            $$ = $2.at(0);
          }
        }
      } else {
        $1->set_args ($2);
        m_in_func--;
        $$ = $1;
      }
    }

start_function
  : NAME LP
    {
      if (!m_in_for && !m_in_arguments) {
        lexer_expression_mode_on ();
      }
      FunctionExprNode* n = new FunctionExprNode (@$, $1);
      m_in_func++;
      $$ = n;
    }

postfix_expression
  : primary_expression { $$ = $1; }
  | function_call { $$ = $1; }

primary_expression
  : INT
    {
      ExprNode *n = new ExprNode (@$, std::string ($1), LITERAL, INT);
      $$ = n;
    }
  | INT_UNIT
    {
      ExprNode *n = new ExprNode (@$, std::string ($1), LITERAL, INT_UNIT);
      $$ = n;
    }
  | TRUE
    {
      ExprNode *n = new ExprNode (@$, std::string ("1"), LITERAL, BOOL);
      $$ = n;
    }
  | FALSE
    {
      ExprNode *n = new ExprNode (@$, std::string ("0"), LITERAL, BOOL);
      $$ = n;
    }
  | DOUBLE
    {
      ExprNode *n = new ExprNode (@$, std::string ($1), LITERAL, DOUBLE);
      $$ = n;
    }
  | DOUBLE_UNIT
    {
      ExprNode *n = new ExprNode (@$, std::string ($1), LITERAL, DOUBLE_UNIT);
      $$ = n;
    }
  | STRING
    {
      ExprNode *n = new ExprNode (@$, std::string ($1), LITERAL, STRING);
      $$ = n;
    }
  | name_or_path
  {
    name_context_list.pop_back ();
    ExprNode *n;
    if (!$1.empty () && $1.at (0)->get_subpath ().substr(0,3) == "DJN") {
      n = new ExprNode (@$, $1.at (0)->get_subpath (), LITERAL, INT);

    }
    else {
      smala_t type = get_type ($1.at (0)->get_subpath ());
      n = new PathExprNode (@$, new PathNode (@$, $1), type);
    }
    $$ = n;
  }
  | NULL
    {
      ExprNode *n = new ExprNode (@$, "", LITERAL, NULL_VALUE);
      $$ = n;
    }
  | LP assignment_expression RP { $2->set_enclosed_with_parenthesis (true); $$ = $2; }


//------------------------------------------------

connector
  : assignment_expression connector_symbol process_list
    {
      NativeExpressionNode *expr_node = new NativeExpressionNode (@$, $1, $2==0, $2==3, true, $2==2 ? false:true);
      expr_node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      for (int i = 0; i < $3.size (); ++i) {
        expr_node->add_output_node ($3.at (i));
      }
      driver.add_native_expression (expr_node);
      driver.add_node (expr_node);
    }

connector_symbol
  : CONNECTOR { lexer_expression_mode_off (); $$ = 1; }
  | PAUSED_CONNECTOR { lexer_expression_mode_off (); $$ = 0; }
  | LAZY_CONNECTOR { lexer_expression_mode_off (); $$ = 3; }
  | ASSGNT_CONN { lexer_expression_mode_off (); $$ = 2; }

binding
  : binding_src binding_type process_list
    {
      for (int i = 0; i < $3.size (); ++i) {
        CtrlNode *node = new CtrlNode (@$, "Binding", "", $2.first, $2.second);
        Node *in = new Node (@$, PATH, "Name", $1);
        node->set_in (in);
        Node *out = new Node (@$, PATH, "Name", $3.at (i));
        node->set_out (out);
        driver.add_node (node);
        node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      }
    }
  | binding_src binding_type lambda
    {
      /* first build the NativeACtion Component */
      // vector< pair<SmalaType, string> >d_args;
      // d_args.push_back (make_pair (LOCAL_NAME, $3->function_name ()));
      // d_args.push_back (make_pair (INT, "1"));
      // Node *native = new Node (LAMBDA, "NativeAction", "");
      // native->set_args_data (d_args);
      // native->set_path ($3->path_data ());
      driver.add_node ($3);
      $3->set_parent (parent_list.empty()? nullptr : parent_list.back ());

      /* then the binding */
      CtrlNode *node = new CtrlNode (@$, "Binding", "", $2.first, $2.second);
      Node *in = new Node (@$, PATH, "Name", $1);
      node->set_in (in);
      node->set_out ($3);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }
    | binding_src binding_type assignment_sequence {
      CtrlNode *node = new CtrlNode (@$, "Binding", "", $2.first, $2.second);
      Node *in = new Node (@$, PATH, "Name", $1);
      node->set_in (in);
      PathNode *path = new PathNode(@$);
      path->add_subpath (new SubPathNode (@$, $3->name(), START));
      Node *out = new Node (@$, PATH, "Name", path);
      node->set_out (out);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    }

causal_dep
  : name_or_path CAUSAL_DEP name_or_path
  {
    CausalDependencyNode *node = new CausalDependencyNode (@$, new PathNode (@1, $1), new PathNode (@1, $3));
    node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
    driver.add_node (node);
  }

assignment_sequence
  : start_assignment_sequence assignment_list RCB
    { 
      driver.add_node (new Node (@$, END_CONTAINER));
      parent_list.pop_back ();
      $$ = $1;
    }

start_assignment_sequence
  : LCB
    {
      std::string loc_name ("loc_ass_seq_" + std::to_string(loc_node_num++));
      Node *node = new Node (@$, SIMPLE, "AssignmentSequence", loc_name);
      driver.add_node (node);

      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());

      node->add_arg (new ExprNode (@$, "1", LITERAL, BOOL));

      node->set_node_type (CONTAINER);
      parent_list.push_back (node);
      cur_node = node;
      $$ = node;
    }
  | NAME COLON LCB
  {
      add_sym (@$, $1, PROCESS);
      std::string loc_name ("loc_ass_seq_" + std::to_string(loc_node_num++));
      Node *node = new Node (@$, SIMPLE, "AssignmentSequence", loc_name);
      driver.add_node (node);

      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());

      node->add_arg (new ExprNode (@$, "1", LITERAL, BOOL));

      node->set_node_type (CONTAINER);
      node->set_name ($1);
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
    if ($1->get_expr_node_type () == PATH_EXPR) {
      $$ = $1->get_path ();
    } else {
      NativeExpressionNode *expr_node = new NativeExpressionNode (@$, $1, false, false, true, false);
      expr_node->set_parent (parent_list.empty()? nullptr : parent_list.back ());

      // build a local bool that will serve as an output for a connector
      // and an input for the binding in construction
      std::string loc_name ("loc_bool");
      loc_name.append (to_string(loc_node_num++));
      std::vector<SubPathNode*> path;
      path.push_back (new SubPathNode (@$, loc_name, START));
      Node *node = new Node (@$, SIMPLE, "Bool", loc_name);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      ExprNode *n = new ExprNode (@$, "0", LITERAL, BOOL);
      node->add_arg (n);

      expr_node->add_output_node (new PathNode (@$, path));
      driver.add_native_expression (expr_node);
      driver.add_node (expr_node);
      path.push_back (new SubPathNode (@$, "true", ITEM));
      loc_name.append(".true");
      $$ = new PathNode (@$, path);
    }
  }

binding_type
  : ARROW { $$ = make_pair ("true", "true"); lexer_expression_mode_off (); }
  | NOT_ARROW { $$ = make_pair ("false", "true"); lexer_expression_mode_off (); }
  | NOT_ARROW_NOT { $$ = make_pair ("false", "false"); lexer_expression_mode_off (); }
  | ARROW_NOT { $$ = make_pair ("true", "false"); lexer_expression_mode_off (); }

lambda
  : start_lambda LCB statement_list RCB
    {
      pop_sym_table ();
      m_in_lambda = false;
      driver.add_node (new Node (@$, END_NATIVE));
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
      SmalaNative *native = new SmalaNative (@$, new_name, "_src_", new PathNode (@2, $2));
      driver.add_node (native);
      $$ = new NativeComponentNode (@$, new_name, nullptr, new PathNode (@2, $2), "1", SIMPLE_ACTION);
      m_in_lambda = true;
      push_sym_table ();
      add_sym (@$, $2.back()->get_subpath (), PROCESS);
    }
  | NAME COLON LP name_or_path RP
    {
      add_sym (@$, $1, PROCESS);
      name_context_list.pop_back ();
      driver.start_debug ();
      driver.in_preamble ();
      string new_name ("func_" + std::to_string (func_num++));
      SmalaNative *native = new SmalaNative (@$, new_name, "_src_", new PathNode (@4, $4));
      driver.add_node (native);
      NativeComponentNode *n = new NativeComponentNode (@$, new_name, nullptr, new PathNode (@4, $4), "1", SIMPLE_ACTION);
      n->set_name ($1);
      $$ = n;
      m_in_lambda = true;
      push_sym_table ();
      add_sym (@$, $4.back()->get_subpath (), PROCESS);
    }

assignment
  : assignment_expression assignment_symbol process_list is_model
    {
      NativeExpressionNode *expr_node = new NativeExpressionNode (@$, $1, $2==1, $2==2, false, $4);
      expr_node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      for (int i = 0; i < $3.size (); ++i) {
        Node *out = new Node (@$, PATH, "Name", $3.at (i));
        expr_node->add_output_node ($3.at (i));
      }
      driver.add_native_expression (expr_node);
      driver.add_node (expr_node);
    }


assignment_symbol
  : ASSIGNMENT { $$ = 0; lexer_expression_mode_off (); }
  | PAUSED_ASSIGNMENT { $$ = 1; lexer_expression_mode_off (); }
  | LAZY_ASSIGNMENT { $$ = 2; lexer_expression_mode_off (); }

is_model
  : %empty { $$ = false; }
  | COLON INT { if ($2.compare ("0") == 0) $$ = false; else $$ = true; }

process_list
  : name_or_path { name_context_list.pop_back (); process_list.clear (); process_list.push_back (new PathNode (@$, $1)); $$ = process_list;}
  | process_list COMMA name_or_path
    {
      name_context_list.pop_back ();
      process_list.push_back (new PathNode (@$, $3));
      $$ = process_list;
    }



//------------------------------------------------

add_children_to
  : start_add_children_to LCB statement_list RCB
    {
      driver.add_node (new Node (@$, END_CONTAINER));
      parent_list.pop_back ();
      //m_in_add_children = false;
      --m_in_add_children;
    }
  | start_add_children_to LCB pname_list RCB
    {
      for (auto name: name_list) {
        $1->add_child (name);
      }
      driver.add_node (new Node (@$, END_CONTAINER));
      parent_list.pop_back ();
      --m_in_add_children;
    }

start_add_children_to
  : ADD_CHILDREN_TO name_or_path
    {
      name_context_list.pop_back ();
      AddChildrenToNode *n = new AddChildrenToNode (@$, "addChildrenTo", new PathNode (@2, $2));
      n->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      parent_list.push_back (n);
      driver.add_node (n);
      //m_in_add_children = true;
      ++m_in_add_children;
      $$ = n;
    }

fsm
  : fsm_decl fsm_items
    {      
      driver.add_node (new Node (@$, END_CONTAINER));
      SetParentNode *p_node = new SetParentNode (@$, $1);
      driver.add_node (p_node);
      parent_list.pop_back ();
      pop_sym_table ();
    }

fsm_decl
  : FSM NAME
    {
      Node *node = new Node  (@$, FSM, "FSM", $2);
      node->set_ignore_parent (true);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      parent_list.push_back (node);
      driver.add_node (node);
      if (driver.debug()) driver.new_line(@$);
      add_sym (@$, $2, PROCESS);
      push_sym_table ();
      $$ = node;
    }

fsm_items
  : LCB fsm_state_list fsm_transition_list RCB

fsm_state_list
  : state
  | fsm_state_list state

state
  : state_decl
  | state_decl start_fsm_state statement_list end_fsm_state
  | state_decl start_fsm_state end_fsm_state

state_decl
  : STATE NAME
    {
      Node *node = new Node (@$, SIMPLE, "FSMState", $2);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      driver.add_node (node);
      add_sym (@$, $2, PROCESS);
      cur_node = node;
      $$ = node;
    }

start_fsm_state
  : LCB
    {
      push_sym_table ();
      cur_node->set_node_type (CONTAINER);
      parent_list.push_back (cur_node);
    }
end_fsm_state
  : RCB 
    {
      driver.add_node (new Node (@$, END_CONTAINER));
      parent_list.pop_back ();
      pop_sym_table ();
    }

fsm_transition_list
  : %empty
  | fsm_transition_list transition

transition
  : NAME ARROW NAME LP name_or_path RP
    {
      name_context_list.pop_back ();
      TransitionNode *node = new TransitionNode (@$, "FSMTransition", "", $1, $3, new PathNode (@5, $5), nullptr);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      if (driver.debug()) driver.new_line(@$);
    }
  | NAME ARROW NAME LP name_or_path COMMA name_or_path RP
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      TransitionNode *node = new TransitionNode (@$, "FSMTransition", "", $1, $3, new PathNode (@5, $5), new PathNode (@7, $7));

      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      if (driver.debug()) driver.new_line(@$);
    }
  | NAME COLON NAME ARROW NAME LP name_or_path COMMA name_or_path RP
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      TransitionNode *node = new TransitionNode (@$, "FSMTransition", $1, $3, $5, new PathNode (@7, $7), new PathNode (@9, $9));

      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      if (driver.debug()) driver.new_line(@$);
    }
  | NAME COLON NAME ARROW NAME LP name_or_path RP
    {
      name_context_list.pop_back ();
      TransitionNode *node = new TransitionNode (@$, "FSMTransition", $1, $3, $5, new PathNode (@7, $7), nullptr);
      driver.add_node (node);
      node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
      if (driver.debug()) driver.new_line(@$);
    }
  | LCB state_list RCB ARROW NAME LP name_or_path RP
    {
      name_context_list.pop_back ();
      for (auto s: $2) {
        TransitionNode *node = new TransitionNode (@$, "FSMTransition", "", s, $5, new PathNode (@7, $7), nullptr);
        driver.add_node (node);
        node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
        if (driver.debug()) driver.new_line(@$);
      }
    }
  | LCB state_list RCB ARROW NAME LP name_or_path COMMA name_or_path RP
    {
      name_context_list.pop_back ();
      name_context_list.pop_back ();
      for (auto s: $2) {
        TransitionNode *node = new TransitionNode (@$, "FSMTransition", "", s, $5, new PathNode (@7, $7), new PathNode (@9, $9));
        driver.add_node (node);;

        driver.add_node (node);
        node->set_parent (parent_list.empty()? nullptr : parent_list.back ());
        if (driver.debug()) driver.new_line(@$);
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
