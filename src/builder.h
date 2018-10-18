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

#include <fstream>
#include <vector>
#include "error_location.h"
#include "activator_node.h"
#include "ctrl_node.h"
#include "type_manager.h"
#include "ast.h"
#include "new_line_node.h"


namespace Smala {

  class error_level
  {
  public:
    typedef enum {info, warning, error} level_t;
  };

  class BuildNode
  {
  public:
    BuildNode (const std::string &name, std::map<std::string, std::string>* sym_table);
    BuildNode (const std::string &name);
    BuildNode ();
    virtual ~BuildNode ();
    std::string get_symbol (const std::string &key) const;
    std::map<std::string, std::string>* sym_table ();
    const std::string& name () const;
    int add_entry (const std::string &key, const std::string &value);
  private:
    std::map<std::string, std::string> *m_sym_table;
    std::string m_name;
  };

  class Builder
  {
  public:
    Builder () :
        m_curloc (nullptr), m_indent (0), m_cpnt_num (0), m_var_num (0), m_error (
            0), m_type_manager (nullptr)
    {
    }
    virtual ~Builder () {};
    virtual int build (const Ast &ast, const std::string &builddir, const std::string &prefix) = 0;
    string filename () { return m_filename; }
  protected:
    smala::ErrorLocation *m_curloc;
    int m_indent, m_cpnt_num, m_var_num, m_error;
    std::string m_null_symbol, m_null_string, m_filename;
    TypeManager *m_type_manager;
    Ast m_ast;
    std::vector<BuildNode> m_parent_list;
    std::map<std::string, std::string> m_types;
    void indent (std::ofstream &os);
    std::string get_constructor (const std::string &type);
    virtual const std::pair<std::string, std::string> parse_symbol (const std::string &symbol);
    void print_error_message (error_level::level_t level, const std::string& message, int error);
    void build_node (std::ofstream &os, Node *node);
    void build_preamble (std::ofstream &os);
    void build_arg_node (std::ofstream &os, Node *node);
    void build_native_code (std::ofstream &os, Node *node);
    void build_cat (std::ofstream &os, Node *node);
    void build_control_node (std::ofstream &os, Node *n);
    void print_start_component (std::ofstream &os, const std::string &name, const std::string &constructor);
    virtual void build_activator (std::ofstream &os, ActivatorNode *node) {}
    virtual void build_set_string (std::ofstream &os, const std::string &cpnt_name, const std::string &spec, const std::string &value) = 0;
    virtual void print_component_decl (std::ofstream &os, const std::string &name) = 0;
    virtual void print_component_constructor (std::ofstream &os, const std::string &constructor) = 0;
    virtual void print_type (std::ofstream &os, ParamType type) = 0;
    virtual void build_use (std::ofstream &os, std::string use) = 0;
    virtual void build_import (std::ofstream &os, std::string import) = 0;
    virtual void build_main_node (std::ofstream &os) = 0;
    virtual void build_define_node (std::ofstream &os, Node *node) = 0;
    virtual void build_end_define (std::ofstream &os, Node *node) = 0;
    virtual void build_smala_native (std::ofstream &os, Node *n) = 0;
    virtual void build_native_action_component (std::ofstream &os, Node *n) = 0;
    virtual void build_native_action (std::ofstream &os, Node *n) = 0;
    virtual void build_instruction (std::ofstream &os, Node *n) = 0;
    void print_find_component (std::ofstream &os, const std::string&, const std::string&);
    virtual std::string build_find_component (const std::string&, const std::string&) = 0;
    virtual void set_property (std::ofstream &os, Node *n) = 0;
    virtual void get_property (std::ofstream &os, Node *n) = 0;
    virtual void alias (std::ofstream &os, Node *n) = 0;
    virtual void merge (std::ofstream &os, Node *n) = 0;
    virtual void remove (std::ofstream &os, Node *n) = 0;
    virtual void move (std::ofstream &os, Node *n, char c) {};
    virtual void repeat (std::ofstream &os, Node *n) = 0;
    virtual void load_xml (std::ofstream &os, Node *n) = 0;
    virtual void add_child (std::ofstream &os, Node *n) = 0;
    virtual void fetch_add_child (std::ofstream &os, std::string &parent, std::string &child, std::string &name) = 0;
    virtual void add_children_to (std::ofstream &os, Node *n) = 0;
    virtual void find (std::ofstream &os, Node *n) = 0;
    virtual void clone (std::ofstream &os, Node *node) {}
    virtual void build_transition_node (std::ofstream &os, CtrlNode *ctrl) = 0;
    std::string build_simple_node (std::ofstream &os, Node *n, bool ignore_parent);
    virtual void build_this_node (std::ofstream &os, Node *n) = 0;
    virtual void build_binary_node (std::ofstream &os, Node *n) = 0;
    virtual void build_unary_node (std::ofstream &os, Node *n) = 0;
    virtual void build_new_line (std::ofstream &os, NewLineNode *n) {
        os << "#line " << n->_line_number << " \"" << n->_filename << "\"" << std::endl;
    }
    
  };

}
