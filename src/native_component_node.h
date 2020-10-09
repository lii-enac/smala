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

  typedef enum native_type {
    SIMPLE_ACTION,ASYNC_ACTION,COLLECTION_ACTION
  } native_type;

  class NativeComponentNode : public Node
  {
  public:
    NativeComponentNode (const location& loc);
    NativeComponentNode (const location& loc, const std::string &action_name, PathNode *path_list, PathNode *path_data, const std::string &is_model, native_type is_async);
    virtual ~NativeComponentNode ();

    const std::string& function_name () const;
    PathNode* path_data () const;
    PathNode* path_list () const;
    const std::string& is_model () const;
    native_type get_native_type () { return m_type; }

  private:
    std::string m_function_name;
    PathNode *m_path_list, *m_path_data;
    std::string m_is_model;
    native_type m_type;
  };

} /* namespace Smala */
