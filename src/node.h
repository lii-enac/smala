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

#include <string>
#include <vector>
#include "error_location.h"
#include "location.hh"

namespace Smala {

  typedef enum
  {
    DELETE, DELETE_CONTENT, DUMP, NOTIFY, RUN, STOP, XMLSERIALIZE, UNKNOWN
  } instruction_t;

  typedef enum SmalaType {
    BOOL, INT, DOUBLE, STRING, PROCESS, CAST_STRING, NULL_VALUE, NAME, LOCAL_NAME, VOID, UNDEFINED
  } smala_t;

  enum NodeType
  {
    ADD_CHILD,
    ADD_CHILDREN_TO,
    ALIAS,
    BREAK,
    CAUSAL_DEP,
    CONTAINER,
    CONTROL,
    DASH_ARRAY,
    DECREMENT,
    END_ADD_CHILD,
    END_ASSIGNMENT,
    END_BLOCK,
    END_CONTAINER,
    END_DEFINE,
    END_ELSE,
    END_IF,
    END_LOOP,
    END_MAIN,
    END_NATIVE,
    END_PROPERTY,
    END_REPEAT,
    EXPR_NODE,
    FOR,
    FOR_EVERY,
    FSM,
    GET_PROPERTY,
    IMPORT,
    INCREMENT,
    INSTRUCTION,
    LAMBDA,
    LOCAL_NODE,
    MERGE,
    MOVE_AFTER,
    MOVE_BEFORE,
    MOVE_END,
    MOVE_FIRST,
    NATIVE_ACTION_CPNT,
    NATIVE_ACTION,
    NATIVE_COLLECTION_ACTION,
    NATIVE_CODE,
    NATIVE_EXPRESSION,
    NATIVE_JAVA,
    NEW_LINE,
    NEW_VAR,
    PATH,
    PRINT,
    RANGE,
    REMOVE,
    REPEAT,
    SELF_SET_PROPERTY,
    SET_PARENT,
    SET_PROPERTY,
    SIMPLE,
    SMALA_NATIVE,
    START_DEFINE,
    START_ELSE,
    START_ELSEIF,
    START_IF,
    START_MAIN,
    SUBPATH,
    TERM_NODE,
    TEXT,
    THIS,
    TRANSITION,
    USE,
    WHILE
  };

class ExprNode;
class PathNode;

typedef void* user_data_t;

class Node
{
public:
    Node (const location& loc, NodeType type);
    Node (const location& loc, NodeType type, const std::string &value, const std::string &name, const std::vector< std::pair<SmalaType, std::string> > &arguments);
    Node (const location& loc, NodeType type, const std::string &value, const std::string &name);
    Node (const location& loc, NodeType type, const std::string &value, PathNode* path);

    void set_node_type (NodeType type);
    void set_build_name (const std::string& build_name);
    void set_parent (Node *p);
    void set_args (std::vector<ExprNode*> args) { m_expr_args = args; }
    void add_arg (ExprNode *arg) { m_expr_args.push_back (arg); }
    std::vector<ExprNode*> get_args () { return m_expr_args; }
    Node* parent ();
    std::string build_name () const;
    NodeType node_type () const;
    virtual ~Node ();
    
    std::string djnn_type () const;
    void set_djnn_type (const std::string& type);
    const std::string& name () const;
    void set_name (const std::string &name);
    std::vector< std::pair<SmalaType, std::string> > args () const;
    void add_args (std::vector< std::pair<smala_t, std::string> > &args);
    void set_error_location (smala::ErrorLocation *loc);
    smala::ErrorLocation* error_location ();
    bool ignore_parent () { return m_ignore_parent; }
    void set_ignore_parent (bool ignore) { m_ignore_parent = ignore; }
    bool has_arguments ();
    bool has_path () { return m_has_path; }
    bool duplicate_warning ();
    void set_duplicate_warning (bool v);
    bool in_expression () { return m_in_expression; }
    void set_in_expression (bool v) { m_in_expression = v; }
    void set_args_spec (std::vector< std::pair<smala_t, std::string> > args) { m_args_spec = args; }
    std::vector< std::pair<smala_t, std::string> >& get_args_spec () { return m_args_spec; }
    PathNode* get_path () { return m_path; }
    void set_path (PathNode* p) { m_path = p; }
    void set_user_data (user_data_t data) { m_data = data; }
    user_data_t get_user_data () { return m_data; }
    void set_keep_name (bool keep) { m_keep_name = keep; }
    bool keep_name () { return m_keep_name; }
    const location& get_location () { return m_loc; }
    void set_is_define_or_main () { m_is_define_or_main = true; }
    bool is_define_or_main () const { return m_is_define_or_main; }
 private:
    bool m_ignore_parent, m_has_path;
    Node *m_parent;
    PathNode *m_path;
    std::string m_djnn_type;
    std::string m_name;
    std::string m_build_name;
    std::vector< std::pair<smala_t, std::string> > m_args_spec;
    std::vector<ExprNode*> m_expr_args;
    bool m_duplicate_warning, m_in_expression, m_keep_name;
    smala::ErrorLocation* m_location;
    NodeType m_node_type;
    user_data_t m_data;
    class location m_loc;
    bool m_is_define_or_main;
};

}
