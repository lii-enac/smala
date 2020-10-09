/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

#include "set_parent_node.h"

namespace Smala
{

  SetParentNode::SetParentNode (const location& loc) :
      Node (loc, SET_PARENT), m_to_attach (nullptr)
  {
  }

  SetParentNode::SetParentNode (const location& loc, 
      Node* to_attach) :
      Node (loc, SET_PARENT), m_to_attach (to_attach)
  {
  }

  SetParentNode::~SetParentNode ()
  {}

  Node*
  SetParentNode::to_attach ()
  {
    return m_to_attach;
  }

  void
  SetParentNode::set_to_attach (Node *n)
  {
    m_to_attach = n;
  }
} /* namespace Smala */
