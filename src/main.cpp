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

using namespace Smala;
using namespace std;

#ifndef __has_include
static_assert(false, "__has_include not supported");
#else
#  if __has_include(<filesystem>)
#    include <filesystem>
namespace filesystem = filesystem;
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
namespace filesystem = experimental::filesystem;
#  elif __has_include(<boost/filesystem.hpp>)
#    include <boost/filesystem.hpp>
namespace filesystem = boost::filesystem;
#  endif
#endif

int main (int argc, const char **argv) {
	Argument arg (argc, argv);
	Driver driver (arg.debug());
	Builder *b = new CPPBuilder ();
	int error = 0;

	int sz = arg.files().size();

	for (auto filename: arg.files()) {
		ifstream is (filename) ;
		if (is.is_open ()) {
			filesystem::path path = filesystem::current_path() / filesystem::path(filename);
			std::string str_path (path);
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
			error |= b->build (driver.ast (), arg.get_option("builddir"), prefix);
			if (error) {
			  remove (b->filename ().c_str());
			}
		} else {
			std::cerr << "could not open " << filename << std::endl;
			error = 1;
			break;
		}
	}
	return error;
}
