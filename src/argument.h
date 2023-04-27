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
#include <vector>
#include <string>

namespace Smala {

  enum Language {
    CPP, JS
  };

class Argument {
public:
	Argument (int argc, const char** argv);
	virtual ~Argument();

	const std::vector<std::string>& files () const;
	const std::map<std::string, std::string>& options () const;

	Language lang () { return _lang; }
	bool debug () { return _debug; }
	bool cleaner () { return _cleaner; }
	bool fastcomp () { return _fastcomp; }
	bool build_html () { return _build_html; };
	const std::string& get_target () const { return m_target_name; }
  	std::string get_option (const std::string& key);
	void usage ();

private:
	std::vector<std::string> m_filenames;
	std::map<std::string, std::string> m_options;
	Language _lang;
	std::string m_program_name, m_target_name;
	bool _debug, _build_html, _cleaner, _fastcomp;
};

} /* namespace Smala */
