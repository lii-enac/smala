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

  enum ArgType
  {
    SYMBOL,
    VALUE,
    STRING_VALUE,
    VAR,
    END,
    SMALA_NULL,
    NONE
  };

class ArgNode : public Node
{
public:
    ArgNode () : Node (), m_arg_type (NONE), m_value ("") { set_node_type (ARG_NODE); };
    ArgNode (ArgType type, const std::string &value) : Node (), m_arg_type (type), m_value (value) { set_node_type (ARG_NODE); };
    ArgType arg_type () const { return m_arg_type; };
    const std::string& arg_value () { return m_value; };
    ~ArgNode () {}

private:
    ArgType m_arg_type;
    std::string m_value;
};

}
