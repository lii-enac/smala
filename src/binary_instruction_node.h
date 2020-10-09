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
#include "path_node.h"

namespace Smala
{

  class BinaryInstructionNode : public Node
  {
  public:
    BinaryInstructionNode (const location& loc, NodeType type);
    BinaryInstructionNode (const location& loc, NodeType type, PathNode* left_arg, PathNode* right_arg);
    virtual ~BinaryInstructionNode ();

    PathNode* left_arg ();
    PathNode* right_arg ();

  private:
    PathNode* m_left_arg;
    PathNode* m_right_arg;
  };

} /* namespace Smala */
