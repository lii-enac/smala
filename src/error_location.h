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
