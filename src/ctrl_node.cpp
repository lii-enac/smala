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

#include "ctrl_node.h"

namespace Smala
{

  CtrlNode::CtrlNode () :
      Node (), m_in (nullptr), m_out (nullptr), m_in_act ("true"), m_out_act ("true")
  {
    set_node_type (CONTROL);
  }

  CtrlNode::CtrlNode (
      const std::string &type, const std::string &name,
      const std::vector<std::pair<ParamType, std::string> > &arguments) :
      Node (type, name, arguments), m_in (nullptr), m_out (nullptr), m_in_act ("true"), m_out_act ("true")
  {
    set_node_type (CONTROL);
  }

  CtrlNode::CtrlNode (const std::string &type, const std::string &name, std::string in_act, std::string out_act) : Node (type, name), m_in (nullptr), m_out (nullptr), m_in_act (in_act), m_out_act (out_act)
  {
    set_node_type (CONTROL);
  }

  CtrlNode::~CtrlNode ()
  {}

  Node*
  CtrlNode::in () const
  {
    return m_in;
  }

  void
  CtrlNode::set_in (Node *in)
  {
    m_in = in;
  }

  Node*
  CtrlNode::out () const
  {
    return m_out;
  }

  void
  CtrlNode::set_out (Node *out)
  {
    m_out = out;
  }
} /* namespace Smala */
