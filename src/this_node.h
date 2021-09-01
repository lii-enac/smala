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

  class ThisNode : public Node
  {
  public:
    ThisNode (const location& loc, const std::string& name) : Node (loc, THIS), _inherit (false), _super_class (nullptr) { set_name (name); }
    virtual ~ThisNode () {}
    void set_inherit (bool v) { _inherit = v; }
    bool inherit () { return _inherit; }
    void set_super_class (ExprNode *super_class_call) { _super_class = super_class_call; }
    ExprNode* get_super_class () { return _super_class; }

  private:
    bool _inherit;
    ExprNode* _super_class;
  };

} /* namespace Smala */
