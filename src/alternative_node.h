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
#include "ctrl_node.h"

namespace Smala
{

  class AlternativeNode : public Node
  {
  public:
    AlternativeNode ();
    virtual ~AlternativeNode ();
    void add_ctrl_node (CtrlNode* n);
    void add_ctrl_nodes (std::vector<CtrlNode*> nodes);
    std::vector<CtrlNode*> ctrl_nodes ();

  private:
    std::vector<CtrlNode*> m_ctrl_nodes;
  };

} /* namespace Smala */
