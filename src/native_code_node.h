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

  class NativeCodeNode : public Node
  {
  public:
    NativeCodeNode ();
    NativeCodeNode (const std::string &code);
    virtual ~NativeCodeNode ();

    const std::string& code () const;

  private:

    std::string m_code;
  };

} /* namespace Smala */
