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
 *
 */

#pragma once

#include "builder.h"

namespace Smala {

  class JSBuilder : public Builder
  {
  public:
    JSBuilder ();
    virtual ~JSBuilder ();
    int build (const Ast &ast, const std::string &builddir, const std::string &prefix, bool debug) override;
  private:
    void build_define (const std::string &prefix);
    void build_use (std::ostream &os, std::string use) override;
    void build_causal_dep (std::ostream &os, Node* node) override;
    void build_import (std::ostream &os, Node* n) override;
    void build_post_import (std::ostream &os) override;
    void build_start_if (std::ostream &os, Node* n) override;
    std::string build_expr (ExprNode *e, expr_production_t prod_t = undefined_t, bool build_fake = false) override;
    void build_instruction (std::ostream &os, Node *node) override;
    void build_for (std::ostream &os, Node *node) override;
    void build_for_every (std::ostream &os, Node *node) override;
    void build_while (std::ostream &os, Node *node) override;
    void build_print (std::ostream &os, Node *node) override;
    std::string build_step (ExprNode *node) override;
    std::string build_fake_name (PathNode* n, bool out);
    std::string build_find (PathNode* n, bool ignore_cast);
    std::string build_path (PathNode* n);
    void set_property (std::ostream &os, Node *node) override;
    void self_set_property (std::ostream &os, Node *n) override;
    void alias (std::ostream &os, Node *node) override;
    void merge (std::ostream &os, Node *node) override;
    void remove (std::ostream &os, Node *node) override;
    void move (std::ostream &os, Node *n, const std::string &c) override;
    void add_child (std::ostream &os, Node *node) override;
    void fetch_add_child (std::ostream &os, const std::string &parent, const std::string &child, const std::string &name) override;
    void build_end_add_child (std::ostream &os, Node *n) override;
    void add_children_to (std::ostream &os, Node *node) override;
    void build_control_node (std::ostream &os, Node *n) override;
    void build_multi_control_node (std::ostream &os, NativeExpressionNode *node);
    void build_native_action_component (std::ostream &os, Node *n) override;
    void build_native_action (std::ostream &os, Node *n) override;
    void build_native_collection_action (std::ostream &os, Node *n) override;
    void build_native_expression (std::ostream &os, Node *n) override;
    void emit_not_a_property (std::ostream &os, const std::string& arg, const std::string& e);
    void build_native_expression_node (std::ostream &os, Node *n) override;
    void build_main_node (std::ostream &os) override;
    void build_end_main (std::ostream &os, Node *node) override;
    void build_this_node (std::ostream &os, Node *node) override;
    void build_define_node (std::ostream &os, Node *node) override;
    void build_end_define (std::ostream &os, Node *node) override;
    void build_transition_node (std::ostream &os, Node *ctrl) override;
    void build_smala_native (std::ostream &os, Node *node) override;
    void build_component (std::ostream &os, const std::string &var_name, const std::string &constructor, std::string &parent_name, std::string &child_name, Node* n) override;
    void build_component_arguments (std::ostream &os, std::string &p_name, std::string &name, Node* n) override;
    void build_range_node (std::ostream &os, Node *node, const string& new_name) override;
    void print_start_component (std::ostream &os, const std::string &name, const std::string &constructor) override;
    // void build_new_line (std::ostream &os, NewLineNode *n) override {
    //     //Builder::build_new_line (os, n);
    //     //os << "Context::instance()->new_line(" << n->_line_number << ", \"" << n->_filename << "\");" << std::endl;
    // }
    void build_simple_control_node (std::ostream &os, NativeExpressionNode *n);
    void print_native_code (std::ostream &os);
    void print_component_decl (std::ostream &os, const std::string &name) override;
    void print_component_constructor (std::ostream &os, const std::string &constructor) override;
    void print_type (std::ostream &os, SmalaType type, ExprNode* arg = 0) override;
    std::map<std::string, std::string> m_import_types;
    void set_location (std::ostream &os, Node *n, bool debug=false) override { if (debug) os << "\n//#line " << n->get_location().begin.line << std::endl; }
    void end_line (std::ostream &os) override { os << ";\n"; }
  private:
    bool m_display_initialized;
    int m_expr_in, m_expr_out;
  };

} /* namespace Smala */
