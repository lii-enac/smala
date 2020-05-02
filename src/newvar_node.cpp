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
      ParamType type, const std::string &name, bool keep_name) :
      Node (NEW_VAR), m_type (type), m_var_name (name), m_keep_name (keep_name)
  {
  }

  NewVarNode::NewVarNode () :
      Node (NEW_VAR), m_type (VOID), m_var_name ("")
  {
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
