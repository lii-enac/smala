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

namespace Smala
{

  class InstructionNode : public Node
  {
  public:
    InstructionNode ();
    InstructionNode (const std::string& name);
    virtual ~InstructionNode ();

    void add_cpnt (std::string&);
    std::vector<std::string>& cpnt_list ();

  private:
    std::vector<std::string> m_cpnt_list;
  };

} /* namespace Smala */
