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

#include "argument.h"
#include <iostream>

namespace Smala
{

  Argument::Argument (int argc, const char** argv) :
    _lang (CPP), _debug (false)
    {
    int i;
    m_program_name = argv[0];
    for (i = 1; i < argc; i++) {
      std::string str (argv[i]);
      if (str.size() > 4 && str.compare (str.size () - 4, 4, ".sma") == 0) {
        m_filenames.push_back (str);
      } else if (str.compare ("-g") == 0) {
         _debug = true;
      }
    }
    if (m_filenames.empty ()) {
      usage ();
    }
  }

  std::string
  Argument::get_option (const std::string& key)
  {
    std::map<std::string, std::string>::const_iterator it;
    it = m_options.find (key);
    if (it == m_options.end ()) {
      return "";
    } else
    return it->second;
  }

  Argument::~Argument ()
  {
    // TODO Auto-generated destructor stub
  }

  const std::vector<std::string>&
  Argument::files () const
  {
    return m_filenames;
  }

  const std::map<std::string, std::string>&
  Argument::options () const
  {
    return m_options;
  }

  void
  Argument::usage ()
  {
    std::cout << "USAGE: smala " <<  "<filenames> [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -g \t enable debug\n";
  }
} /* namespace Smala */
