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

#include "range_node.h"

namespace Smala
{

  RangeNode::RangeNode (const std::string& name, const std::string& lower_arg, bool left_open, const std::string& upper_arg, bool right_open) :
      Node (), m_lower_arg (lower_arg), m_upper_arg (upper_arg), m_left_open (left_open), m_right_open (right_open)
  {
  }

  RangeNode::RangeNode () :
      Node (), m_lower_arg (""), m_upper_arg (""), m_left_open (false), m_right_open (false)
  {
  }

  RangeNode::~RangeNode ()
  {}

} /* namespace Smala */
