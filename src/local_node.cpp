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

#include "local_node.h"

namespace Smala
{

  LocalNode::LocalNode () :
      Node (), m_root (nullptr), m_path ("")
  {

  }

  LocalNode::LocalNode (
      Node* root, const std::string &path) :
      Node (), m_root (root), m_path (path)
  {
  }

  LocalNode::~LocalNode ()
  {}

  Node*
  LocalNode::root () const
  {
    return m_root;
  }

  void
  LocalNode::set_root (Node *root)
  {
    m_root = root;
  }

  const std::string&
  LocalNode::path() const
  {
    return m_path;
  }

  void
  LocalNode::set_path (const std::string &path)
  {
    m_path = path;
  }
} /* namespace Smala */
