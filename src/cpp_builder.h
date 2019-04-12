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
    int build (const Ast &ast, const std::string &builddir, const std::string &prefix) override;
  private:
    void build_define (const std::string &prefix);
    void build_use (std::ofstream &os, std::string use) override;
    void build_import (std::ofstream &os, std::string import) override;
    void build_instruction (std::ofstream &os, Node *node) override;
    void build_arg_node (std::ofstream &os, Node *node) override;
    void build_set_string (std::ofstream &os, const std::string &cpnt_name, const std::string &spec, const std::string &value) override;
    void build_for (std::ofstream &os, Node *node) override;
    void build_while (std::ofstream &os, Node *node) override;
    void build_step (std::ofstream &os, Node *node, bool is_incr) override;
    void set_property (std::ofstream &os, Node *node) override;
    bool known_symbol (const string &name);
    void set_ref_property (std::ofstream &os, Node *node);
    void end_property (std::ofstream &os, Node *n) override;
    void get_property (std::ofstream &os, Node *node) override;
    void alias (std::ofstream &os, Node *node) override;
    void merge (std::ofstream &os, Node *node) override;
    void remove (std::ofstream &os, Node *node) override;
    void move (std::ofstream &os, Node *n, int c) override;
    void repeat (std::ofstream &os, Node *node) override;
    void load_xml (std::ofstream &os, Node *node) override;
    void add_child (std::ofstream &os, Node *node) override;
    void fetch_add_child (std::ofstream &os, std::string &parent, std::string &child, std::string &name) override;
    void add_children_to (std::ofstream &os, Node *node) override;
    void find (std::ofstream &os, Node *node) override;
    void clone (std::ofstream &os, Node *node) override;
    void build_control_node (std::ofstream &os, Node *n) override;
    void build_activator (std::ofstream &os, ActivatorNode *n) override;
    void build_native_action_component (std::ofstream &os, Node *n) override;
    void build_native_action (std::ofstream &os, Node *n) override;
    void build_native_expression (std::ofstream &os, Node *n) override;
    void build_native_expression_node (std::ofstream &os, Node *n) override;
    void build_main_node (std::ofstream &os) override;
    void build_this_node (std::ofstream &os, Node *node) override;
    void build_define_node (std::ofstream &os, Node *node) override;
    void build_end_define (std::ofstream &os, Node *node) override;
    void build_binary_node (std::ofstream &os, Node *node) override;
    void build_unary_node (std::ofstream &os, Node *node) override;
    void build_transition_node (std::ofstream &os, CtrlNode *ctrl) override;
    void build_smala_native (std::ofstream &os, Node *node) override;
    void build_new_line (std::ofstream &os, NewLineNode *n) override {
        Builder::build_new_line (os, n);
        os << "Context::instance()->new_line(" << n->_line_number << ", \"" << n->_filename << "\");" << std::endl;
    }
    void build_simple_control_node (std::ofstream &os, NativeExpressionNode *n);
    void build_dash_array (std::ofstream &os, DashArrayNode *n) override;
    void check_and_build_connector (std::ofstream &os, Node *n, const std::string &name, const std::string &side);
    void print_native_code (std::ofstream &os);
    void print_component_decl (std::ofstream &os, const std::string &name) override;
    void print_component_constructor (std::ofstream &os, const std::string &constructor) override;
    std::string build_find_component (const std::string&, const std::string&) override;
    void print_type (std::ofstream &os, ParamType type) override;
    void print_args (std::ofstream &os, std::vector< std::pair<ParamType, std::string> > args, bool is_first);
    std::map<std::string, std::string> m_import_types;
  private:
    bool m_in_for;
  };

} /* namespace Smala */
