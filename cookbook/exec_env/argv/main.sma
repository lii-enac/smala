/*
*  Smala cookbook argv
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2021-2022)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Stephane Conversy <stephane.conversy@enac.fr>
*
*/
use core
use gui

_native_code_
%{

#include <iostream>

int
init (int argc, char** argv)
{
    std::cerr << "==== args" << std::endl;
    for (int i=0; i<argc; ++i) {
        std::cerr << argv[i] << std::endl;
    }

    return 0;
}


%}

_main_
Component root {
    init (argc, argv)
    Frame f ("argv", 0, 0, 500, 500)
    Exit ex (0, 1)
    f.close -> ex
}
