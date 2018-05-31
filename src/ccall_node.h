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

  class NativeCallNode : public Node
  {
  public:
    NativeCallNode ();
    NativeCallNode (ParamType return_type, const std::string &name, const std::string &func_name, std::vector< std::pair<ParamType, std::string> > &arguments);
    NativeCallNode (ParamType return_type, const std::string &name, const std::string &func_name);
    virtual ~NativeCallNode ();

    std::string& func_name ();
    ParamType return_type ();

  private:
    std::string m_func_name;
    ParamType m_return_type;
  };

} /* namespace Smala */
