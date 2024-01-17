#pragma once

#include <iostream>

namespace smala
{
  class ErrorLocation
  {
  public:
    ErrorLocation (const std::string &file, unsigned int l, unsigned int pos) :
        m_file (file), m_l (l), m_pos (pos) {}
    ~ErrorLocation () {};
    std::string& file () { return m_file; }
    unsigned int line () { return m_l; }
    unsigned int position () { return m_pos; }

  private:
    std::string m_file;
    unsigned int m_l, m_pos;
  };
}
