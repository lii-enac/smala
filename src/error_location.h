/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2018-2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Magnaudet Mathieu <mathieu.magnaudet@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
#pragma once

#include <iostream>

namespace smala
{
  class ErrorLocation
  {
  public:
    ErrorLocation (const std::string &file, unsigned int l, unsigned int pos) :
        m_file (file)/*, m_l (l), m_pos (pos)*/ {}
    ~ErrorLocation () {};
    std::string& file () { return m_file; }
    // unsigned int line () { return m_l; }         // 2024.01 : cut off for now -- maybe to be removed
    // unsigned int position () { return m_pos; }   // 2024.01 : cut off for now -- maybe to be removed

  private:
    std::string m_file;
    //unsigned int m_l, m_pos;                      // 2024.01 : cut off for now -- maybe to be removed
  };
}
