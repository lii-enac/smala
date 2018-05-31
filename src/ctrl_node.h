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

  class CtrlNode : public Node
  {
  public:
    CtrlNode ();
    CtrlNode (const std::string &type, const std::string &name, const std::vector< std::pair<ParamType, std::string> > &arguments);
    CtrlNode (const std::string &type, const std::string &name);
    virtual ~CtrlNode ();

    Node* in () const;
    void set_in (Node *in);
    Node* out () const;
    void set_out (Node *out);

  private:
    Node *m_in;
    Node *m_out;
  };

} /* namespace Smala */
