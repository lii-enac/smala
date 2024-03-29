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
    NewVarNode (const location& loc);
    NewVarNode (const location& loc, SmalaType type, const std::string &name, bool keep_name = false);
    virtual ~NewVarNode ();

    std::string& var_name ();
    SmalaType type ();
    bool keep_name () { return m_keep_name; }

  private:
    SmalaType m_type;
    std::string m_var_name;
    bool m_keep_name;
  };

} /* namespace Smala */
