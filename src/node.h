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

namespace Smala {

  typedef enum
  {
    DELETE, DUMP, NOTIFY, RUN, STOP, UNKNOWN
  } instruction_t;

  enum ParamType {
    INT, DOUBLE, STRING, NAME, LOCAL_NAME
  };

  enum NodeType
  {
    ACTIVATOR,
    ADD_CHILD,
    ADD_CHILDREN_TO,
    ALIAS,
    ALTERNATIVE,
    ARG_NODE,
    BINARY_OP,
    CAT,
    CCALL,
    CLONE,
    CONTAINER,
    CONTROL,
    DASH_ARRAY,
    END_CONTAINER,
    END_DEFINE,
    END_NATIVE,
    END_PROPERTY,
    END_REPEAT,
    FIND,
    FSM,
    GET_PROPERTY,
    IMPORT,
    INSTRUCTION,
    LITERAL,
    LOAD_XML,
    LOCAL_NODE,
    MACRO,
    MERGE,
    MOVE_AFTER,
    MOVE_BEFORE,
    MOVE_END,
    MOVE_FIRST,
    NATIVE_ACTION,
    NATIVE_ACTION_CPNT,
    NATIVE_CODE,
    NATIVE_EXPRESSION,
    NATIVE_JAVA,
    NEW_LINE,
    PATH,
    REMOVE,
    REPEAT,
    SET_PARENT,
    SET_PROPERTY,
    SIMPLE,
    SMALA_NATIVE,
    START_DEFINE,
    START_MAIN,
    SWITCH,
    TEXT,
    THIS,
    UNARY_OP,
    USE
  };

class Node
{
public:
    Node ();
    Node (const std::string &type, const std::string &name, const std::vector< std::pair<ParamType, std::string> > &arguments);
    Node (const std::string &type, const std::string &name);
    void set_node_type (NodeType type);
    void set_build_name (const std::string& build_name);
    void set_parent (Node *p);
    Node* parent ();
    std::string build_name () const;
    NodeType node_type () const;
    virtual ~Node ();
    
    std::string djnn_type () const;
    void set_djnn_type (const std::string& type);
    const std::string& name () const;
    void set_name (const std::string &name);
    std::vector< std::pair<ParamType, std::string> > args () const;
    void add_args (std::vector< std::pair<ParamType, std::string> > &args);
    void set_location (smala::ErrorLocation *loc);
    smala::ErrorLocation* location ();
    bool has_arguments ();
    void set_has_arguments (bool v);
    bool duplicate_warning ();
    void set_duplicate_warning (bool v);
    bool in_expression () { return m_in_expression; }
    void set_in_expression (bool v) { m_in_expression = v; }
private:
    Node * m_parent;
    std::string m_djnn_type;
    std::string m_name;
    std::string m_build_name;
    std::vector< std::pair<ParamType, std::string> > m_args;
    bool m_has_arguments, m_duplicate_warning, m_in_expression;
    smala::ErrorLocation* m_location;
    NodeType m_node_type;
};

}
