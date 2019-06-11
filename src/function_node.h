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

  class FunctionNode : public Node
  {
  public:
    FunctionNode ();
    FunctionNode (ParamType return_type, const std::string &name, const std::string &func_name);
    FunctionNode (const std::string &func_name);
    virtual ~FunctionNode ();

    std::string& func_name ();
    std::string& return_name ();
    ParamType return_type ();

  private:
    std::string m_func_name;
    std::string m_return_name;
    ParamType m_return_type;
  };

} /* namespace Smala */
