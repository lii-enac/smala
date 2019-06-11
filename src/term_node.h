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

namespace Smala {

  enum ArgType
  {
    CAST_DOUBLE,
    CAST_STRING,
    CAST_PROCESS,
    END_LCB_BLOCK,
    END,
    NONE,
    SMALA_NULL,
    START_LCB_BLOCK,
    STRING_VALUE,
    SYMBOL,
    VALUE,
    VAR
  };

class TermNode : public Node
{
public:
    TermNode () : Node (), m_arg_type (NONE), m_value (""), m_in_func (false) { set_node_type (TERM_NODE); };
    TermNode (ArgType type, const std::string &value) : Node (), m_arg_type (type), m_value (value), m_in_func (false) { set_node_type (TERM_NODE); };
    void set_arg_type (ArgType type) { m_arg_type = type; }
    ArgType arg_type () const { return m_arg_type; };
    const std::string& arg_value () { return m_value; };
    void set_in_func (bool v) { m_in_func = v; }
    bool is_in_func () { return m_in_func; }
    ~TermNode () {}

private:
    ArgType m_arg_type;
    std::string m_value;
    bool m_in_func;
};

}
