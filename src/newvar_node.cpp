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

#include "newvar_node.h"

namespace Smala
{

  NewVarNode::NewVarNode (
      ParamType type, const std::string &name) :
      Node (), m_type (type), m_var_name (name)
  {
    set_node_type (NEW_VAR);
  }

  NewVarNode::NewVarNode () :
      Node (), m_type (VOID), m_var_name ("")
  {
    set_node_type (NEW_VAR);;
  }

  NewVarNode::~NewVarNode ()
  {}

  std::string&
  NewVarNode::var_name ()
  {
    return m_var_name;
  }

  ParamType
  NewVarNode::type ()
  {
    return m_type;
  }
} /* namespace Smala */
