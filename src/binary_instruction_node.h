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

  class BinaryInstructionNode : public Node
  {
  public:
    BinaryInstructionNode ();
    BinaryInstructionNode (const std::string& left_arg, const std::string& right_arg);
    virtual ~BinaryInstructionNode ();

    std::string& left_arg ();
    std::string& right_arg ();

  private:
    std::string m_left_arg;
    std::string m_right_arg;
  };

} /* namespace Smala */
