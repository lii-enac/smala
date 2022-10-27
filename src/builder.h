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

#include <sstream>
#include <vector>
#include "error_location.h"
#include "ctrl_node.h"
#include "type_manager.h"
#include "ast.h"
#include "new_line_node.h"

//#define DBG ;
#define DBG cerr << __FILE__ << ":" << __LINE__ << endl;

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
    const string& filename () const { return m_filename; }
  
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

    void build_preamble (std::ostream &os, bool debug=false);
    virtual void build_use (std::ostream &os, std::string use) = 0;
    virtual void build_import (std::ostream &os, Node* n) = 0;
    virtual void build_post_import (std::ostream &os) {};

    virtual void build_smala_native (std::ostream &os, Node *n) = 0;
    virtual void build_end_native (std::ostream &os) { os << "}\n\n"; }

    virtual void build_define_node (std::ostream &os, Node *node) = 0;
    virtual void build_end_define (std::ostream &os, Node *node) = 0;
    
    virtual void build_main_node (std::ostream &os) = 0;
    virtual void build_end_main (std::ostream &os, Node *node) = 0;

    void build_node (std::ostream &os, Node *node);
    std::string build_simple_node (std::ostream &os, Node *n);
    
    void extract_leaves (std::vector<ExprNode*> &leaves, ExprNode *n);
    virtual std::string build_expr (ExprNode* n, expr_production_t prod_t = undefined_t, bool build_fake = false) = 0;
    virtual void build_properties (std::ostream &os) {}

    virtual void build_native_expression (std::ostream &os, Node *n) {}
    virtual void build_native_expression_node (std::ostream &os, Node *n) {};

    virtual void build_control_node (std::ostream &os, Node *n) = 0;
    virtual void build_causal_dep (std::ostream &os, Node* node) = 0;

    virtual void build_component (std::ostream &os, const std::string &var_name, const std::string &constructor, std::string &parent_name, std::string &child_name, Node* n) = 0;
    virtual void print_start_component (std::ostream &os, const std::string &var_name, const std::string &constructor) = 0;
    virtual void build_component_arguments (std::ostream &os, std::string &parent_name, std::string &child_name, Node* n) = 0;
    virtual void print_component_decl (std::ostream &os, const std::string &name) = 0;
    virtual void print_component_constructor (std::ostream &os, const std::string &constructor) = 0;
    virtual void print_type (std::ostream &os, SmalaType type, ExprNode* arg = 0) = 0;
    virtual void build_array_var (std::ostream &os, ArrayVarNode* a) {}
    
    virtual void build_start_if (std::ostream &os, Node* n) = 0;
    virtual void build_start_else (std::ostream &os) { os << "else {\n"; }
    virtual void build_start_else_if (std::ostream &os) { os << "else "; }
    virtual void build_end_block (std::ostream &os) {  os << "}\n"; }

    virtual void build_for (std::ostream &os, Node *node) {}
    virtual void build_for_every (std::ostream &os, Node *node) {}
    void build_for_node (std::ostream &os, Node *node); // generic behavior for for and for_every
    virtual std::string build_step (ExprNode *node) = 0;
    virtual void build_while (std::ostream &os, Node *node) {}
    virtual void build_break (std::ostream &os, Node *n) { os << n->djnn_type() << ";\n"; }
    
    virtual void build_native_action_component (std::ostream &os, Node *n) = 0;
    virtual void build_native_action (std::ostream &os, Node *n) = 0;
    virtual void build_native_collection_action (std::ostream &os, Node *n) = 0;

    virtual void build_range_node (std::ostream &os, Node *node, const string& new_name) = 0; // SwitchRange
    virtual void build_transition_node (std::ostream &os, Node *ctrl) = 0; // FSM transition
    virtual void build_this_node (std::ostream &os, Node *n) = 0;

    virtual void set_property (std::ostream &os, Node *n) = 0;
    virtual void self_set_property (std::ostream &os, Node *n) = 0;

    virtual void build_instruction (std::ostream &os, Node *n) = 0; // dump, serialize, run, stop etc.
    virtual void alias (std::ostream &os, Node *n) = 0;
    virtual void merge (std::ostream &os, Node *n) = 0;
    virtual void remove (std::ostream &os, Node *n) = 0;
    virtual void move (std::ostream &os, Node *n, const string& c) {};
    virtual void add_child (std::ostream &os, Node *n) = 0;
    virtual void build_end_add_child (std::ostream &os, Node* n) = 0;
    virtual void fetch_add_child (std::ostream &os, const std::string &parent, const std::string &child, const std::string &name) {};
    virtual void add_children_to (std::ostream &os, Node *n) = 0;
    virtual void build_print (std::ostream &os, Node *node) {}

    void indent (std::ostream &os);
    virtual void set_location (std::ostream &os, Node *n, bool debug=false) {}
    virtual void end_line (std::ostream &os) = 0;
    void print_error_message (error_level::level_t level, const std::string& message, int error);
    
    // scope context
    virtual void push_ctxt (const string& parent_name = string());
    virtual void pop_ctxt ();
    
    // helpers
    std::string get_constructor (const std::string &type);
    bool is_string (ExprNode *e);
    bool has_complex_term (PathNode *n);

    
  };

}
