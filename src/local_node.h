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

  class LocalNode : public Node
  {
  public:
    LocalNode ();
    LocalNode (Node *root, const std::string &path);
    virtual ~LocalNode ();

    Node* root () const;
    void set_root (Node *root);
    const std::string& path () const;
    void set_path (const std::string &path);


  private:
    Node* m_root;
    std::string m_path;
  };

} /* namespace Smala */
