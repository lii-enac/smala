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
#include "term_node.h"

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
    void set_has_argument (bool v) { m_has_argument = v; }
    bool has_argument () { return m_has_argument; }
    void set_type (instruction_t type) { m_type = type; }
    void set_args (std::vector<TermNode*> &expression) { m_expression = expression; m_has_argument = true; }
    std::vector<TermNode*>& args () { return m_expression; }
    instruction_t type () { return m_type; }
  private:
    std::vector<std::string> m_cpnt_list;
    bool m_has_argument;
    std::vector<TermNode*> m_expression;
    instruction_t m_type;
  };

} /* namespace Smala */
