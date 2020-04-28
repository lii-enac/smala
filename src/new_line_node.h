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

class NewLineNode : public Node {
public:

  NewLineNode (const std::string& filename, int line_number)
  : Node(NEW_LINE), _filename (filename), _line_number (line_number) {}

//private:
  std::string _filename;
  int _line_number;
};

}
