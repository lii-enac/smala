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

#include "node.h"

namespace Smala
{

  class ActivatorNode : public Node
  {
  public:
    ActivatorNode (Node* n) : Node (), m_node (n) { set_node_type (ACTIVATOR); }
    virtual ~ActivatorNode () {}
    void set_node (Node* n) { m_node = n; };
    Node* node () { return m_node; }

  private:
    Node* m_node;
  };

} /* namespace Smala */
