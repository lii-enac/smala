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

#include "operator_node.h"

namespace Smala
{

  OperatorNode::OperatorNode () :
      Node (), m_left (nullptr), m_right (nullptr)
  {

  }

  OperatorNode::OperatorNode (
      const std::string &type, const std::string &name,
      const std::vector<std::pair<ParamType, std::string> > &arguments) :
      Node (type, name, arguments), m_left (nullptr), m_right (nullptr)
  {
  }

  OperatorNode::OperatorNode (const std::string &type, const std::string &name) : Node (type, name), m_left (nullptr), m_right (nullptr)
  {
  }

  OperatorNode::~OperatorNode ()
  {}

  Node*
  OperatorNode::left () const
  {
    return m_left;
  }

  void
  OperatorNode::set_left (Node *left)
  {
    m_left = left;
  }

  Node*
  OperatorNode::right () const
  {
    return m_right;
  }

  void
  OperatorNode::set_right (Node *right)
  {
    m_right = right;
  }
} /* namespace Smala */
