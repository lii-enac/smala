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

#include <map>
#include <string>
#include "type_manager.h"

namespace Smala {

  class CPPTypeManager : public TypeManager
  {
  public:
    typedef std::map<std::string, std::string> symtable_t;
    CPPTypeManager ();
    ~CPPTypeManager ();
  };
}
