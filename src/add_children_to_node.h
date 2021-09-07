/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2021)
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

  class AddChildrenToNode : public Node
  {
  public:
    AddChildrenToNode (const location& loc, const std::string &value, PathNode* path) : Node (loc, ADD_CHILDREN_TO, value, path) {};
    virtual ~AddChildrenToNode () {}

    void add_child (PathNode* child) { m_children.push_back (child); }
    std::vector <PathNode* > children () { return m_children; }

  private:
    std::vector <PathNode*> m_children;
  };

} /* namespace Smala */