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

  class NativeComponentNode : public Node
  {
  public:
    NativeComponentNode ();
    NativeComponentNode (const std::string &action_name, PathNode *path_data, const std::string &is_model, bool is_async);
    virtual ~NativeComponentNode ();

    const std::string& function_name () const;
    PathNode* path_data () const;
    const std::string& is_model () const;
    bool is_async () { return m_is_async; }

  private:
    std::string m_function_name;
    PathNode* m_path_data;
    std::string m_is_model;
    bool m_is_async;
  };

} /* namespace Smala */
