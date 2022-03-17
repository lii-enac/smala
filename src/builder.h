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
#include "dash_array_node.h"
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

  typedef enum {string_t, number_t, process_t, undefined_t} expr_production_t;


  class BuildNode
  {
  public:
    BuildNode (const std::string &name, BuildNode* prev_node);
    BuildNode (const std::string &name);
    BuildNode ();
    virtual ~BuildNode ();
    std::string get_symbol (const std::string &key) const;
    std::string get_key (const std::string &value) const;
    std::map<std::string, std::string>* sym_table ();
    const std::string& name () const;
    int add_entry (const std::string &key, const std::string &value);
  private:
    std::map<std::string, std::string> *m_sym_table;
    BuildNode *m_prev_node;
    std::string m_name;
  };

  class Builder
  {
  public:
    Builder () :
        m_curloc (nullptr), m_indent (0), m_cpnt_num (0), m_var_num (0), m_error (
            0), m_sym_num (0), m_native_num (0), m_in_for (false), m_in_set_text (false), m_in_set_property(false) , m_in_native_action (false), m_in_static_expr (false), m_in_switch (false),
            m_after_else (false), m_type_manager (nullptr), m_define_or_main_node (nullptr), m_in_code (false)
    {
    }
    virtual ~Builder () {};
    virtual int build (const Ast &ast, const std::string &builddir, const std::string &prefix, bool debug) = 0;
    string filename () { return m_filename; }
  protected:
    smala::ErrorLocation *m_curloc;
    int m_indent, m_cpnt_num, m_var_num, m_error, m_sym_num, m_native_num;
    bool m_in_for, m_in_set_text, m_in_set_property, m_in_native_action, m_in_static_expr, m_in_switch, m_after_else, m_in_code;
    std::string m_null_symbol, m_null_string, m_filename;
    TypeManager *m_type_manager;
    Ast m_ast;
    std::string m_cur_building_name;
    std::vector<BuildNode*> m_parent_list;
    std::map<std::string, std::string> m_types;
    Node * m_define_or_main_node;

    virtual void pop_ctxt ();
    virtual void push_ctxt ();
    void extract_leaves (std::vector<ExprNode*> &leaves, ExprNode *n);
    void indent (std::ofstream &os);
    std::string get_constructor (const std::string &type);
    void print_error_message (error_level::level_t level, const std::string& message, int error);
    void build_node (std::ofstream &os, Node *node);
    void build_for_node (std::ofstream &os, Node *node);
    void build_preamble (std::ofstream &os, bool debug=false);
    bool is_string (ExprNode *e);
    bool has_complex_term (PathNode *n);
    virtual void build_control_node (std::ofstream &os, Node *n) = 0;
    virtual void print_start_component (std::ofstream &os, const std::string &name, const std::string &constructor) = 0;
    virtual void build_component_arguments (std::ostream &os, std::string &p_name, std::string &name, Node* n) = 0;
    virtual void print_component_decl (std::ofstream &os, const std::string &name) = 0;
    virtual void print_component_constructor (std::ofstream &os, const std::string &constructor) = 0;
    virtual std::string build_expr (ExprNode* n, expr_production_t prod_t = undefined_t, bool build_fake = false) = 0;
    virtual void print_type (std::ofstream &os, SmalaType type) = 0;
    virtual void build_causal_dep (std::ofstream &os, Node* node) = 0;
    virtual void build_use (std::ofstream &os, std::string use) = 0;
    virtual void build_import (std::ofstream &os, Node* n) = 0;
    virtual void build_post_import (std::ofstream &os) {};
    virtual void build_main_node (std::ofstream &os) = 0;
    virtual void build_define_node (std::ofstream &os, Node *node) = 0;
    virtual void build_end_define (std::ofstream &os, Node *node) = 0;
    virtual void build_end_main (std::ofstream &os, Node *node) = 0;
    virtual void build_smala_native (std::ofstream &os, Node *n) = 0;
    virtual void build_start_else (std::ofstream &os) { os << "else {\n"; }
    virtual void build_start_else_if (std::ofstream &os) { os << "else "; }
    virtual void build_start_if (std::ofstream &os, Node* n) = 0;
    virtual void build_end_block (std::ofstream &os) {  os << "}\n"; }
    virtual void build_end_native (std::ofstream &os) { os << "}\n\n"; }
    virtual void build_break (std::ostream &os, Node *n) { os << n->djnn_type() << ";\n"; }
    virtual void build_for (std::ofstream &os, Node *node) {}
    virtual void build_for_every (std::ofstream &os, Node *node) {}
    virtual void build_while (std::ofstream &os, Node *node) {}
    virtual void build_print (std::ofstream &os, Node *node) {}
    virtual std::string build_step (ExprNode *node) = 0;
    virtual void build_native_action_component (std::ofstream &os, Node *n) = 0;
    virtual void build_native_action (std::ofstream &os, Node *n) = 0;
    virtual void build_native_collection_action (std::ofstream &os, Node *n) = 0;
    virtual void build_native_expression (std::ofstream &os, Node *n) {}
    virtual void build_native_expression_node (std::ofstream &os, Node *n) {};
    virtual void build_instruction (std::ofstream &os, Node *n) = 0;
    virtual void build_range_node (std::ofstream &os, Node *node, const string& new_name) = 0;
    virtual void set_property (std::ofstream &os, Node *n) = 0;
    virtual void self_set_property (std::ofstream &os, Node *n) = 0;
    virtual void alias (std::ofstream &os, Node *n) = 0;
    virtual void merge (std::ofstream &os, Node *n) = 0;
    virtual void remove (std::ofstream &os, Node *n) = 0;
    virtual void move (std::ofstream &os, Node *n, const string& c) {};
    virtual void add_child (std::ofstream &os, Node *n) = 0;
    virtual void build_end_add_child (std::ofstream &os, Node* n) = 0;
    virtual void fetch_add_child (std::ofstream &os, const std::string &parent, const std::string &child, const std::string &name) {};
    virtual void add_children_to (std::ofstream &os, Node *n) = 0;
    virtual void build_transition_node (std::ofstream &os, Node *ctrl) = 0;
    std::string build_simple_node (std::ofstream &os, Node *n);
    virtual void build_this_node (std::ofstream &os, Node *n) = 0;
    virtual void set_location (std::ofstream &os, Node *n, bool debug=false) {}
    virtual void end_line (std::ofstream &os) = 0;
    // virtual void build_new_line (std::ofstream &os, NewLineNode *n) {
    //     os << "#line " << n->_line_number << " \"" << n->_filename << "\"" << std::endl;
    // }
    virtual void build_dash_array (std::ofstream &os, DashArrayNode *n) {}
  };

}
