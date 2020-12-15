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

  InstructionNode::InstructionNode (const location& loc, const std::string& name) :
      Node (loc, INSTRUCTION), m_type (UNKNOWN)
  {
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

  InstructionNode::InstructionNode (const location& loc) :
      Node (loc, INSTRUCTION), m_type (UNKNOWN)
  {
  }

  InstructionNode::~InstructionNode ()
  {}

  void
  InstructionNode::add_path (PathNode* path)
  {
    m_path_list.push_back (path);
  }

  std::vector<PathNode*>&
  InstructionNode::path_list ()
  {
    return m_path_list;
  }

} /* namespace Smala */
