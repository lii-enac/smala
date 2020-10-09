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

#include "transition_node.h"

namespace Smala
{

  TransitionNode::TransitionNode (const location& loc) :
      Node (loc, TRANSITION), m_src (""), m_dst (""), m_trigger (nullptr), m_action (nullptr)
  {
  }

  TransitionNode::TransitionNode (const location& loc, const std::string &type, const std::string &name, const std::string &src_name, const std::string &dst_name, PathNode* trigger, PathNode* action) :
      Node (loc, TRANSITION, type, name), m_src (src_name), m_dst (dst_name), m_trigger (trigger), m_action (action)
  {
  }

} /* namespace Smala */
