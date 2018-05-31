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

#include "native_code_node.h"

namespace Smala
{

  NativeCodeNode::NativeCodeNode () :
      Node (), m_code ("")
  {
    set_node_type (NATIVE_CODE);
  }

  NativeCodeNode::NativeCodeNode (const std::string &code) :
      Node (), m_code (code)
  {
    set_node_type (NATIVE_CODE);
  }

  NativeCodeNode::~NativeCodeNode ()
  {}

  const std::string&
  NativeCodeNode::code() const
  {
    return m_code;
  }
} /* namespace Smala */
