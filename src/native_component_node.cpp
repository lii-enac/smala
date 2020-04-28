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

#include "native_component_node.h"

namespace Smala
{

  NativeComponentNode::NativeComponentNode () :
      Node (NATIVE_ACTION_CPNT), m_function_name (""), m_path_data (nullptr), m_is_model ("1")
  {
    set_djnn_type ("NativeAction");
  }

  NativeComponentNode::NativeComponentNode (
      const std::string &action_name, PathNode *path, const std::string &is_model) :
      Node (NATIVE_ACTION_CPNT), m_function_name (action_name), m_path_data (path), m_is_model (is_model)
  {
    set_djnn_type ("NativeAction");
  }

  NativeComponentNode::~NativeComponentNode ()
  {}

  const std::string&
  NativeComponentNode::function_name() const
  {
    return m_function_name;
  }

  PathNode*
  NativeComponentNode::path_data() const
  {
    return m_path_data;
  }

  const std::string&
  NativeComponentNode::is_model() const
  {
    return m_is_model;
  }
} /* namespace Smala */
