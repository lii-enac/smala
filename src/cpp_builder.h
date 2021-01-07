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

  class CPPBuilder : public Builder
  {
  public:
    CPPBuilder ();
    virtual ~CPPBuilder ();
    int build (const Ast &ast, const std::string &builddir, const std::string &prefix, bool debug) override;
  private:
    void build_define (const std::string &prefix);
    void build_use (std::ofstream &os, std::string use) override;
    void build_causal_dep (std::ofstream &os, Node* node) override;
    void build_import (std::ofstream &os, Node* n) override;
    void build_post_import (std::ofstream &os) override;
    void build_instruction (std::ofstream &os, Node *node) override;
    void build_start_if (std::ofstream &os, Node* n) override;
    void build_for (std::ofstream &os, Node *node) override;
    void build_for_every (std::ofstream &os, Node *node) override;
    void build_while (std::ofstream &os, Node *node) override;
    void build_print (std::ofstream &os, Node *node) override;
    std::string build_step (ExprNode *node) override;
    std::string build_expr (ExprNode *e, expr_production_t prod_t = undefined_t, bool build_fake = false) override;
    std::string build_fake_name (PathNode* n, bool out);
    std::string build_find (PathNode* n, bool ignore_cast);
    std::string build_path (PathNode* n);
    void set_property (std::ofstream &os, Node *node) override;
    void alias (std::ofstream &os, Node *node) override;
    void merge (std::ofstream &os, Node *node) override;
    void remove (std::ofstream &os, Node *node) override;
    void move (std::ofstream &os, Node *n, const std::string &c) override;
    void add_child (std::ofstream &os, Node *node) override;
    void fetch_add_child (std::ofstream &os, const std::string &parent, const std::string &child, const std::string &name) override;
    void build_end_add_child (std::ofstream &os) override;
    void add_children_to (std::ofstream &os, Node *node) override;
    void build_control_node (std::ofstream &os, Node *n) override;
    void build_multi_control_node (std::ofstream &os, NativeExpressionNode *node);
    void build_native_action_component (std::ofstream &os, Node *n) override;
    void build_native_action (std::ofstream &os, Node *n) override;
    void build_native_collection_action (std::ofstream &os, Node *n) override;
    void build_native_expression (std::ofstream &os, Node *n) override;
    void emit_not_a_property (std::ofstream &os, const std::string& arg, const std::string& e);
    void build_native_expression_node (std::ofstream &os, Node *n) override;
    void build_main_node (std::ofstream &os) override;
    void build_end_main (std::ofstream &os, Node *node) override;
    void build_this_node (std::ofstream &os, Node *node) override;
    void build_define_node (std::ofstream &os, Node *node) override;
    void build_end_define (std::ofstream &os, Node *node) override;
    void build_transition_node (std::ofstream &os, Node *ctrl) override;
    void build_smala_native (std::ofstream &os, Node *node) override;
    void build_component_arguments (std::ostream &os, std::string &p_name, std::string &name, Node* n) override;
    void build_range_node (std::ofstream &os, Node *node, const string& new_name) override;
    void print_start_component (std::ofstream &os, const std::string &name, const std::string &constructor) override;
    // void build_new_line (std::ofstream &os, NewLineNode *n) override {
    //     //Builder::build_new_line (os, n);
    //     //os << "Context::instance()->new_line(" << n->_line_number << ", \"" << n->_filename << "\");" << std::endl;
    // }
    void build_simple_control_node (std::ofstream &os, NativeExpressionNode *n);
    void build_dash_array (std::ofstream &os, DashArrayNode *n) override;
    void print_native_code (std::ofstream &os);
    void print_component_decl (std::ofstream &os, const std::string &name) override;
    void print_component_constructor (std::ofstream &os, const std::string &constructor) override;
    void print_type (std::ofstream &os, SmalaType type) override;
    std::map<std::string, std::string> m_import_types;
    void set_location (std::ofstream &os, Node *n) override { os << "\n#line " << n->get_location().begin.line << std::endl; }
    void end_line (std::ofstream &os) override { os << ";\n"; }
  private:
    bool m_display_initialized;
    int m_expr_in, m_expr_out;
    std::map<std::string,bool> used_processes;
  };

} /* namespace Smala */
