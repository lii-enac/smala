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
#include "c_builder.h"
#include "j_builder.h"
#include "cpp_builder.h"

using namespace Smala;
using namespace std;

int main (int argc, const char **argv) {
	Argument arg (argc, argv);
	Driver driver;
	Builder *b;
	int error = 0;
	switch (arg.lang ()) {
	  case C:
	    b = new CBuilder ();
	    break;
	  case JAVA:
	    b = new JBuilder (arg.get_option ("package"));
	    break;
	  case CPP:
	    b = new CPPBuilder ();
	}
	int sz = arg.files().size();

	for (auto filename: arg.files()) {
		ifstream is (filename) ;
		if (is.is_open ()) {
			driver.set_stream (&is, filename);
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
		} else {
			std::cerr << "could not open " << filename << std::endl;
			error = 1;
			break;
		}
	}
	return error;
}
