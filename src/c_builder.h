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

  class CBuilder : public Builder
  {
  public:
    CBuilder ();
    virtual ~CBuilder ();
    virtual int build (const Ast &ast, const std::string &builddir, const std::string &prefix);
  private:
    void build_define (const std::string &prefix);
    void build_use (std::ofstream &os, std::string use);
    void build_import (std::ofstream &os, std::string import);
    void build_instruction (std::ofstream &os, Node *node);
    void build_set_string (std::ofstream &os, const std::string &cpnt_name, const std::string &spec, const std::string &value);
    void set_property (std::ofstream &os, Node *node);
    void get_property (std::ofstream &os, Node *node);
    void alias (std::ofstream &os, Node *node);
    void merge (std::ofstream &os, Node *node);
    void remove (std::ofstream &os, Node *node);
    void repeat (std::ofstream &os, Node *node);
    void load_xml (std::ofstream &os, Node *node);
    void add_child (std::ofstream &os, Node *node);
    void fetch_add_child (std::ofstream &os, std::string &parent, std::string &child, std::string &name);
    void add_children_to (std::ofstream &os, Node *node);
    void find (std::ofstream &os, Node *node);
    void build_native_action_component (std::ofstream &os, Node *n);
    void build_native_action (std::ofstream &os, Node *n);
    void build_main_node (std::ofstream &os);
    void build_this_node (std::ofstream &os, Node *node);
    void build_define_node (std::ofstream &os, Node *node);
    void build_end_define (std::ofstream &os, Node *node);
    void build_binary_node (std::ofstream &os, Node *node);
    void build_unary_node (std::ofstream &os, Node *node);
    void build_transition_node (std::ofstream &os, CtrlNode *ctrl);
    void build_smala_native (std::ofstream &os, Node *node);
    void check_and_build_connector (std::ofstream &os, Node *n, const std::string &name, const std::string &side);
    void print_native_code (std::ofstream &os);
    void print_component_decl (std::ofstream &os, const std::string &name);
    void print_component_constructor (std::ofstream &os, const std::string &constructor);
    std::string build_find_component (const std::string&, const std::string&);
    void print_type (std::ofstream &os, ParamType type);
    void print_args (std::ofstream &os, std::vector< std::pair<ParamType, std::string> > args, bool is_first);
  };

} /* namespace Smala */
