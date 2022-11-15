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
