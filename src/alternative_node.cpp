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

#include "alternative_node.h"

namespace Smala
{

  AlternativeNode::AlternativeNode () :
      Node ()
  {
    set_node_type (ALTERNATIVE);
  }

  AlternativeNode::~AlternativeNode ()
  {}

  void
  AlternativeNode::add_ctrl_node (CtrlNode* n)
  {
    m_ctrl_nodes.push_back (n);
  }

  void
  AlternativeNode::add_ctrl_nodes (std::vector<CtrlNode*> nodes)
  {
    m_ctrl_nodes.insert (m_ctrl_nodes.end (), nodes.begin (), nodes.end ());
  }

  std::vector<CtrlNode*>
  AlternativeNode::ctrl_nodes ()
  {
    return m_ctrl_nodes;
  }

} /* namespace Smala */
