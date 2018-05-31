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

#include "ccall_node.h"

namespace Smala
{

  NativeCallNode::NativeCallNode (
      ParamType return_type, const std::string &name,
      const std::string &func_name,
      std::vector<std::pair<ParamType, std::string> > &arguments) :
      Node (), m_return_type (return_type), m_func_name (func_name)
  {
    set_name (name);
    set_node_type (CCALL);
    add_args (arguments);
  }

  NativeCallNode::NativeCallNode (
      ParamType return_type, const std::string &name,
      const std::string &func_name) :
      Node (), m_return_type (return_type), m_func_name (func_name)
  {
    set_name (name);
    set_node_type (CCALL);
  }

  NativeCallNode::NativeCallNode () :
      Node (), m_return_type (INT), m_func_name ("")
  {
    set_node_type (CCALL);
  }

  NativeCallNode::~NativeCallNode ()
  {}

  std::string&
  NativeCallNode::func_name ()
  {
    return m_func_name;
  }

  ParamType
  NativeCallNode::return_type ()
  {
    return m_return_type;
  }
} /* namespace Smala */
