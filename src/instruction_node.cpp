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

#include "instruction_node.h"

namespace Smala
{

  InstructionNode::InstructionNode (const std::string& name) :
      Node ()
  {
    set_node_type (INSTRUCTION);
    set_name (name);
  }

  InstructionNode::InstructionNode () :
      Node ()
  {
  }

  InstructionNode::~InstructionNode ()
  {}

  void
  InstructionNode::add_cpnt (std::string &cpnt)
  {
    m_cpnt_list.push_back (cpnt);
  }

  std::vector<std::string>&
  InstructionNode::cpnt_list ()
  {
    return m_cpnt_list;
  }

} /* namespace Smala */
