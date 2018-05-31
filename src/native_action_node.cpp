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

#include "native_action_node.h"

namespace Smala
{

  NativeActionNode::NativeActionNode () :
      Node (), m_action_name (""), m_param_name (""), m_code ("")
  {
    set_node_type (NATIVE_ACTION);
  }

  NativeActionNode::NativeActionNode (
      const std::string &action_name, const std::string &param_name, const std::string &code) :
      Node (), m_action_name (action_name), m_param_name (param_name), m_code (code)
  {
    set_node_type (NATIVE_ACTION);
  }

  NativeActionNode::~NativeActionNode ()
  {}

  const std::string&
  NativeActionNode::action_name() const
  {
    return m_action_name;
  }

  const std::string&
  NativeActionNode::param_name() const
  {
    return m_param_name;
  }

  const std::string&
  NativeActionNode::code() const
  {
    return m_code;
  }
} /* namespace Smala */
