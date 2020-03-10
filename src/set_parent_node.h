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

#pragma once

#include "node.h"

namespace Smala
{

  class SetParentNode : public Node
  {
  public:
    SetParentNode ();
    SetParentNode (Node *to_attach);
    virtual ~SetParentNode ();

    Node* to_attach ();
    void set_to_attach (Node *to_attach);

  private:
    Node* m_to_attach;
  };

} /* namespace Smala */
