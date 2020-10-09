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

#include "binary_instruction_node.h"

namespace Smala
{

  BinaryInstructionNode::BinaryInstructionNode (const location& loc, NodeType type, PathNode* left_arg, PathNode* right_arg) :
      Node (loc, type), m_left_arg (left_arg), m_right_arg (right_arg)
  {
  }

  BinaryInstructionNode::BinaryInstructionNode (const location& loc, NodeType type) :
      Node (loc, type), m_left_arg (nullptr), m_right_arg (nullptr)
  {
  }

  BinaryInstructionNode::~BinaryInstructionNode ()
  {}

  PathNode*
  BinaryInstructionNode::left_arg ()
  {
    return m_left_arg;
  }

  PathNode*
  BinaryInstructionNode::right_arg ()
  {
    return m_right_arg;
  }

} /* namespace Smala */
