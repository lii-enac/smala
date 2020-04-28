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
#include "node.h"
#include "path_node.h"

namespace Smala {

  enum ArgType
  {
    END_LCB_BLOCK,
    END,
    FUNCTION_CALL,
    NONE,
    SMALA_NULL,
    START_LCB_BLOCK,
    STRING_VALUE,
    SYMBOL,
    OPERATOR,
    VALUE,
    VAR
  };

class TermNode : public Node
{
public:
    TermNode () : Node (TERM_NODE), m_arg_type (NONE), m_path_value (nullptr), m_str_value (""), m_in_func (false) { }
    TermNode (ArgType type, PathNode* value) : Node (TERM_NODE), m_arg_type (type), m_path_value (value), m_str_value (""), m_in_func (false) { }
    TermNode (ArgType type, std::string value) : Node (TERM_NODE), m_arg_type (type), m_path_value (nullptr), m_str_value (value), m_in_func (false) { }
    void set_arg_type (ArgType type) { m_arg_type = type; }
    ArgType arg_type () const { return m_arg_type; };
    PathNode*  path_arg_value () { return m_path_value; };
    std::string& str_arg_value () { return m_str_value; };
    ~TermNode () {}

private:
    ArgType m_arg_type;
    PathNode* m_path_value;
    std::string m_str_value;
    bool m_in_func;
};

}
