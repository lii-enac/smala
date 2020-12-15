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

#include <vector>
#include <string>
#include "node.h"
#include "path_node.h"
namespace Smala
{

  class InstructionNode : public Node
  {
  public:
    InstructionNode (const location& loc);
    InstructionNode (const location& loc, const std::string& name);
    virtual ~InstructionNode ();

    void add_path (PathNode *n);
    std::vector<PathNode*>& path_list ();
    void set_type (instruction_t type) { m_type = type; }
    instruction_t type () { return m_type; }
  private:
    std::vector<PathNode*> m_path_list;
    instruction_t m_type;
  };

} /* namespace Smala */
