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
#include "term_node.h"

namespace Smala
{

  class RangeNode : public Node
  {
  public:
    RangeNode (const location& loc);
    RangeNode (const location& loc, const std::string& name, std::vector <TermNode*> left, bool left_open, std::vector <TermNode*> right, bool right_open);
    virtual ~RangeNode ();

    std::vector <TermNode*> lower_arg () { return m_lower_arg; }
    std::vector <TermNode*> upper_arg () { return m_upper_arg; }
    bool left_open () { return m_left_open; }
    bool right_open () { return m_right_open; }

  private:
    std::vector <TermNode*> m_lower_arg;
    std::vector <TermNode*> m_upper_arg;
    bool m_left_open, m_right_open;
  };

} /* namespace Smala */
