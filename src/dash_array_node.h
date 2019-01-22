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
    DashArrayNode ();
    DashArrayNode (const std::string &name, std::vector<int> pattern) : _pattern (pattern) {
      set_name (name);
      set_node_type (DASH_ARRAY);
    }
    virtual ~DashArrayNode () {}
    std::vector<int>& get_pattern () { return _pattern; }

  private:
    std::vector <int> _pattern;
  };

} /* namespace Smala */
