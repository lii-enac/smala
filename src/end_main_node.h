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

  class EndMainNode : public Node
  {
  public:
    EndMainNode (Node* n) : Node (), m_node (n) { set_node_type (END_MAIN); }
    virtual ~EndMainNode () {}
    void set_node (Node* n) { m_node = n; };
    Node* node () { return m_node; }

  private:
    Node* m_node;
  };

} /* namespace Smala */
