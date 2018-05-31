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

  class NativeActionNode : public Node
  {
  public:
    NativeActionNode ();
    NativeActionNode (const std::string &action_name, const std::string &param_name, const std::string &code);
    virtual ~NativeActionNode ();

    const std::string& action_name () const;
    const std::string& param_name () const;
    const std::string& code () const;


  private:
    std::string m_action_name;
    std::string m_param_name;
    std::string m_code;
  };

} /* namespace Smala */
