We provide a Makefile that should work on MacOS (10.12+), Linux, Windows.

1. install djnn-cpp (see https://github.com/lii-enac/djnn-cpp/blob/master/INSTALL.md)
2. open a terminal (a mingw64 one on windows, not an MSYS2 one), cd to the parent of the djnn-cpp directory and copy/paste the following commands:
```
git clone https://github.com/lii-enac/smala.git  
cd smala  
make install-pkgdeps  
make -j4  
```
3. test it with the following commands:
```
make -j simplest_test
```
4. try any other cookbook listed in smala/cookbook subdirectories
```
make -j pan_and_zoom_test
make -j fitts_law_test
```
etc.

5. copy [cookbook/stand_alone](cookbook/stand_alone) somewhere else and start your own project (see [Makefile](cookbook/stand_alone/Makefile) in stand_alone)
