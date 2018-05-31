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

  BinaryInstructionNode::BinaryInstructionNode (const std::string& left_arg, const std::string& right_arg) :
      Node (), m_left_arg (left_arg), m_right_arg (right_arg)
  {
  }

  BinaryInstructionNode::BinaryInstructionNode () :
      Node ()
  {
  }

  BinaryInstructionNode::~BinaryInstructionNode ()
  {}

  std::string&
  BinaryInstructionNode::left_arg ()
  {
    return m_left_arg;
  }

  std::string&
  BinaryInstructionNode::right_arg ()
  {
    return m_right_arg;
  }

} /* namespace Smala */
