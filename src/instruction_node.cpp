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
      Node (), m_has_argument (false), m_type (UNKNOWN)
  {
    set_node_type (INSTRUCTION);
    set_name (name);

    // DELETE, DUMP, NOTIFY, RUN, STOP
    if (name == "Delete")
      set_type (DELETE);
    else if (name == "Dump")
      set_type (DUMP);
    else if (name == "Notify")
      set_type (NOTIFY);
    else if (name == "Run")
      set_type (RUN);
    else if (name == "Stop")
      set_type (STOP);
    else if (name == "XMLSerialize")
      set_type (XMLSERIALIZE);
  }

  InstructionNode::InstructionNode () :
      Node (), m_has_argument (false), m_type (UNKNOWN)
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
