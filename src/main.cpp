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

#include <iostream>
#include <fstream>
#include <vector>
#include "argument.h"
#include "scanner.h"
#include "parser.hpp"
#include "driver.h"
#include "cpp_builder.h"
#include "js_builder.h"
#include "html_builder.h"

#include "core/utils/filesystem.h"

using namespace Smala;
using namespace std;

int main (int argc, const char **argv) {
	Argument arg (argc, argv);
	Driver driver (arg.debug());
	Builder *b;
	if (arg.lang () == JS )
	  b = new JSBuilder ();
	else
	  b = new CPPBuilder ();
	int error = 0;

	int sz = arg.files().size();

	for (auto filename: arg.files()) {
		ifstream is (filename) ;
		if (is.is_open ()) {
			filesystem::path path = filesystem::current_path() / filesystem::path(filename);
			// on some system (Windows, Linux) path as to convert into string ()
			std::string str_path (path.string ());
			// on some system (Windows) str_path need to be converted : C:\foo\bar -> C:/foo/bar
			std::replace (str_path.begin (), str_path.end (), '\\', '/');
			driver.set_stream (&is, str_path);
			error |= driver.parse ();
			is.close ();

			std::string prefix;
			string builddir = arg.get_option("builddir");

			if(!builddir.empty()) { 
				size_t slash_pos = filename.find('/');
				if(slash_pos==std::string::npos) slash_pos=0;
				else ++slash_pos;
				prefix = filename.substr(slash_pos, filename.size() - slash_pos - 4);
			}
			else {
				prefix = filename.substr(0, filename.size() - 4);	
			}
			error |= b->build (driver.ast (), arg.get_option("builddir"), prefix, arg.debug());
			if (error) {
			  remove (b->filename ().c_str());
			}
		} else {
			std::cerr << "could not open " << filename << std::endl;
			error = 1;
			break;
		}
	}
	if (!error) {
	  if (arg.lang () == JS && arg.build_html()) {
      build_html (arg.get_target(), arg.files ());
	  }
	}
	return error;
}
