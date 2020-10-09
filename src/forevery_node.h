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
#include "path_node.h"

namespace Smala
{

  class ForEveryNode : public Node
  {
  public:
    ForEveryNode (const location& loc, const std::string& new_name, PathNode* path) : Node (loc, FOR_EVERY), m_new_name (new_name), m_path (path) {};
    virtual ~ForEveryNode () {}

    std::string& get_new_name () { return m_new_name; }
    PathNode* get_path () const { return m_path; }

  private:
    std::string  m_new_name;
    PathNode* m_path;
  };

} /* namespace Smala */
