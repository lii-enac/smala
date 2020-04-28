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

namespace Smala
{

  class DashArrayNode : public Node
  {
  public:
    DashArrayNode () : Node (DASH_ARRAY) {}
    DashArrayNode (const std::string &name, std::vector<int> pattern) : Node (DASH_ARRAY), _pattern (pattern) {
      set_name (name);
    }
    virtual ~DashArrayNode () {}
    std::vector<int>& get_pattern () { return _pattern; }

  private:
    std::vector <int> _pattern;
  };

} /* namespace Smala */
