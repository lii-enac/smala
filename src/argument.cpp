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
  inline bool ends_with(std::string const & value, std::string const & ending)
  {
      if (ending.size() > value.size()) return false;
      return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
  }

  Argument::Argument (int argc, const char** argv) :
    _lang (CPP), _debug (false), _cleaner (false), _fastcomp (false)
    {
    int i;
    m_program_name = argv[0];
    for (i = 1; i < argc; i++) {
      std::string str (argv[i]);
      if (str.size() > 4 && str.compare (str.size () - 4, 4, ".sma") == 0) {
        m_filenames.push_back (str);
      } else if (str.compare ("-g") == 0) {
        _debug = true;
      } else if (str.compare ("-gen-cleaner") == 0) {
        _cleaner = true;
      } else if (str.compare ("-fastcomp") == 0) {
        _fastcomp = true;
      } else if (str.compare ("-lang=js") == 0) {
          _lang = JS;
      } else if (str.compare ("-o") == 0) {
          std::string fn (argv[i+1]);
          if (ends_with(fn, ".html") || ends_with(fn, ".htm")) {
              _build_html = true;
              m_target_name = fn;
          } else {
              std::cerr << "unknown output file format: " << fn << std::endl;
              exit (0);
          }
          i++;
      } else if (str.compare ("-builddir")==0) {
        if (i<argc)
          m_options["builddir"] = argv[i+1];
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
    std::cout << "  -gen-cleaner \t enable cleaner generator\n";
    std::cout << "  -lang=<cpp|js> \t change the target language\n";
  }
} /* namespace Smala */
