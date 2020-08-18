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

  class CausalDependencyNode : public Node
  {
  public:
    CausalDependencyNode ();
    CausalDependencyNode (PathNode* src, PathNode* dst) : Node (CAUSAL_DEP), m_src (src), m_dst (dst) {}
    virtual ~CausalDependencyNode () {}

    PathNode* src () const { return m_src; }
    PathNode* dst () const { return m_dst; }

  private:
    PathNode *m_src;
    PathNode *m_dst;
  };

} /* namespace Smala */
