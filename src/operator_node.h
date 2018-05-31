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

  class OperatorNode : public Node
  {
  public:
    OperatorNode ();
    OperatorNode (const std::string &type, const std::string &name, const std::vector< std::pair<ParamType, std::string> > &arguments);
    OperatorNode (const std::string &type, const std::string &name);
    virtual ~OperatorNode ();

    Node* left () const;
    void set_left (Node *left);
    Node* right () const;
    void set_right (Node *right);

  private:
    Node* m_left;
    Node* m_right;
  };

} /* namespace Smala */
