/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2020)
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

  class NativeCollectionActionNode : public Node
  {
  public:
	NativeCollectionActionNode (const location& loc) : Node (loc, NATIVE_COLLECTION_ACTION), m_action_name (""), m_param_name (""), m_code ("") {}
	NativeCollectionActionNode (const location& loc, const std::string &action_name, const std::string & param_list, const std::string & param_name, const std::string &code) : Node (loc, NATIVE_COLLECTION_ACTION), m_action_name (action_name), m_param_list (param_list), m_param_name (param_name), m_code (code) {}
    virtual ~NativeCollectionActionNode () {}

    const std::string& action_name () const { return m_action_name;}
    const std::string& list_name () const { return m_param_list; }
    const std::string& param_name () const { return m_param_name; }
    const std::string& code () const { return m_code; }


  private:
    std::string m_action_name;
    std::string m_param_list;
    std::string m_param_name;
    std::string m_code;
  };

} /* namespace Smala */
