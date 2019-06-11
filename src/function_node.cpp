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

#include "function_node.h"

namespace Smala
{

  FunctionNode::FunctionNode (
      ParamType return_type, const std::string &name,
      const std::string &func_name) :
      Node (),m_func_name (func_name), m_return_type (return_type), m_return_name (name)
  {
    set_name (name);
    set_node_type (CCALL);
  }

  FunctionNode::FunctionNode () :
      Node (), m_return_type (VOID), m_func_name (""), m_return_name ("")
  {
    set_node_type (CCALL);
  }

  FunctionNode::FunctionNode (const std::string &func_name) :
      Node (), m_return_type (VOID), m_func_name (func_name), m_return_name ("")
  {
    set_node_type (CCALL);
  }

  FunctionNode::~FunctionNode ()
  {}

  std::string&
  FunctionNode::func_name ()
  {
    return m_func_name;
  }

  std::string&
  FunctionNode::return_name ()
  {
    return m_return_name;
  }

  ParamType
  FunctionNode::return_type ()
  {
    return m_return_type;
  }
} /* namespace Smala */
