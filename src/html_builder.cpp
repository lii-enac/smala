/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

#include <fstream>

#include "html_builder.h"

namespace Smala {

  void
  build_html (const std::string& target, const std::vector<std::string> files)
  {
    std::ofstream os (target);
    os << "<!DOCTYPE>\n";
    os << "<html>\n";
    os <<"\t<meta charset=\"UTF-8\">\n";
    os << "\t<body>\n";
    os << "\t\t<canvas id=\"canvas\"></canvas>\n";

    for (auto f : files) {
        std::string name = f.substr(0, f.size() - 3) + "js";
        os << "\t\t<script src = \"" << name << "\"></script>\n";
    }
    os << "\t\t<script text=\"text/javascript\" src=\"djnn.js\"></script>\n";
    os << "\t\t<script text=\"text/javascript\" src=\"djnn_js_api.js\"></script>\n";
    os << "\t</body>\n";
    os << "</html>\n";
    os.close ();
  }
}


