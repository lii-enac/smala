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

  class NewVarNode : public Node
  {
  public:
    NewVarNode ();
    NewVarNode (ParamType type, const std::string &name, bool keep_name);
    virtual ~NewVarNode ();

    std::string& var_name ();
    ParamType type ();
    bool keep_name () { return m_keep_name; }

  private:
    std::string m_var_name;
    ParamType m_type;
    bool m_keep_name;
  };

} /* namespace Smala */
