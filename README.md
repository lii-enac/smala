Smala is a dedicated language for programming interactive software.

It is the result of the work from the Interactive Informatics team at ENAC <http://lii.recherche.enac.fr>

This software provides a compiler that translates a Smala program into a C++ one. To compile the executable, you need to install the djnn-cpp library: <https://github.com/lii-enac/djnn-cpp> and follow the instructions in [INSTALL.md](INSTALL.md).

More information on Smala can be found here: <http://smala.io>

install dependencies:

```
make install-pkgdeps
```

start a new smala project in ../stand_alone:

```
make stand_alone
```

or

```
make stand_alone stand_alone_dir=/path/to/my/project
```

to start a new project in /path/to/my/project.
