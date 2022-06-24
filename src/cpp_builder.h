/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2018)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *      Stéphane Conversy <stephane.conversy@enac.fr>
 *
 */

#pragma once

#include "builder.h"

namespace Smala {

  class CPPBuilder : public Builder
  {
  public:
    CPPBuilder ();
    virtual ~CPPBuilder ();
    virtual void pop_ctxt () override;
    virtual void push_ctxt () override;
    int build (const Ast &ast, const std::string &builddir, const std::string &prefix, bool debug) override;
  private:
    void build_header (const std::string &prefix);

    void build_use (std::ostream &os, std::string use) override;
    void build_import (std::ostream &os, Node* n) override;
    void build_post_import (std::ostream &os) override;

    void build_smala_native (std::ostream &os, Node *node) override;
    void build_end_native (std::ostream &os) override;

    void build_define_node (std::ostream &os, Node *node) override;
    void build_end_define (std::ostream &os, Node *node) override;

    void build_main_node (std::ostream &os) override;
    void build_end_main (std::ostream &os, Node *node) override;

    std::string build_expr (ExprNode *e, expr_production_t prod_t = undefined_t, bool build_fake = false) override;
    std::string build_expr_rec (ExprNode *e, expr_production_t prod_t = undefined_t, bool build_fake = false);
    void build_properties (std::ostream &os) override;

    void build_native_expression (std::ostream &os, Node *n) override;
    void build_native_expression_node (std::ostream &os, Node *n) override;
    void emit_not_a_property (std::ostream &os, const std::string& arg);

    void build_control_node (std::ostream &os, Node *n) override;
    void build_multi_control_node (std::ostream &os, NativeExpressionNode *node);
    void build_simple_control_node (std::ostream &os, NativeExpressionNode *n);
    void build_causal_dep (std::ostream &os, Node* node) override;

    void build_component (std::ostream &os, const std::string &var_name, const std::string &constructor, std::string &parent_name, std::string &child_name, Node* n) override;
    void print_start_component (std::ostream &os, const std::string &name, const std::string &constructor) override;
    void build_component_arguments (std::ostream &os, std::string &parent_name, std::string &child_name, Node* n) override;
    void print_component_decl (std::ostream &os, const std::string &name) override;
    void print_component_constructor (std::ostream &os, const std::string &constructor) override;
    void print_type (std::ostream &os, SmalaType type) override;

    void build_start_if (std::ostream &os, Node* n) override;
    void build_start_else (std::ostream &os) override;
    void build_start_else_if (std::ostream &os) override;
    void build_end_block (std::ostream &os) override;

    void build_for (std::ostream &os, Node *node) override;
    void build_for_every (std::ostream &os, Node *node) override;
    std::string build_step (ExprNode *node) override;
    void build_while (std::ostream &os, Node *node) override;

    void build_native_action_component (std::ostream &os, Node *n) override;
    void build_native_action (std::ostream &os, Node *n) override;
    void build_native_collection_action (std::ostream &os, Node *n) override;

    void build_transition_node (std::ostream &os, Node *ctrl) override;
    void build_range_node (std::ostream &os, Node *node, const string& new_name) override;
    void build_this_node (std::ostream &os, Node *node) override;
    void build_dash_array (std::ostream &os, DashArrayNode *n) override;

    void set_property (std::ostream &os, Node *node) override;
    void self_set_property (std::ostream &os, Node *n) override;

    void build_instruction (std::ostream &os, Node *node) override;
    void alias (std::ostream &os, Node *node) override;
    void merge (std::ostream &os, Node *node) override;
    void remove (std::ostream &os, Node *node) override;
    void move (std::ostream &os, Node *n, const std::string &c) override;
    void add_child (std::ostream &os, Node *node) override;
    void fetch_add_child (std::ostream &os, const std::string &parent, const std::string &child, const std::string &name) override;
    void build_end_add_child (std::ostream &os, Node *n) override;
    void add_children_to (std::ostream &os, Node *node) override;
    void build_print (std::ostream &os, Node *node) override;

    // helpers
    std::string build_fake_name (PathNode* n, bool out);
    std::string build_find (PathNode* n, bool ignore_cast);
    std::string build_path (PathNode* n);
    void set_location (std::ostream &os, Node *n, bool debug=false) override { if (debug) os << "\n#line " << n->get_location().begin.line << std::endl; }
    void end_line (std::ostream &os) override { os << ";\n"; }

  private:
    bool m_display_initialized;
    int m_expr_in, m_expr_out; // are we in an expression or not?
    std::map<std::string, std::string> m_import_types;

    std::map<std::string,bool> used_processes; // register which component is used, to generate the corrsponding includes
    
    using sym_t = std::map<std::string, std::string>;
    sym_t sym; // contains aliases and properties created to build the djnn tree, and that can be reused
    std::vector<sym_t> sym_stack; // this takes care of scope inside define, main(?), native
    sym_t prop_sym; // like sym, but for properties only
    std::vector<sym_t> prop_sym_stack; // like sym_stack
    sym_t m_new_syms_from_build_expr; // this contains newly dynamic_cast'ed properties while building expressions
  };

} /* namespace Smala */
