/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2021)
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

  class SelfAssignNode : public Node
  {
  public:
    SelfAssignNode (const location& loc, PathNode* path, const std::string& symbol ) : Node (loc, SELF_SET_PROPERTY, "", path), _symbol (symbol){ }
    virtual ~SelfAssignNode () {}
    const std::string& symbol () const { return _symbol; }

  private:
    std::string _symbol;
  };

} /* namespace Smala */
