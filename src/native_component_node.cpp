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
      Node (NATIVE_ACTION_CPNT), m_function_name (""), m_path_list (nullptr), m_path_data (nullptr), m_is_model ("1"), m_type (SIMPLE_ACTION)
  {
    set_djnn_type ("NativeAction");
  }

  NativeComponentNode::NativeComponentNode (
      const std::string &action_name, PathNode *path_list, PathNode *path_data, const std::string &is_model, native_type type) :
      Node (NATIVE_ACTION_CPNT), m_function_name (action_name), m_path_list (path_list), m_path_data (path_data), m_is_model (is_model), m_type (type)
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

  PathNode*
  NativeComponentNode::path_list() const
  {
    return m_path_list;
  }

  const std::string&
  NativeComponentNode::is_model() const
  {
    return m_is_model;
  }
} /* namespace Smala */
