#relative path
src_dir := src
res_dir := res
exe_dir := .

exe := button

srcs_sma := src/Button.sma src/main.sma
# or
#srcs_sma := $(shell find $(src_dir) -name "*.sma")

# native sources
srcs_other := src/foo.cpp
# or
#srcs_other := $(shell find $(src_dir) -name "*.cpp")

# ----- PKGS DEPENDENCIES OR EXTERNAL -----

# external libraries
# CXXFLAGS += $(shell pkg-config --cflags foo)
# LDFLAGS +=
# LIBS += $(shell pkg-config --libs foo)
# 
# OR
# 
# pkgdeps depending on OS
# ifeq ($(os),Linux)
# pkgdeps := #name of the pkgs on apt linux
# endif

# ifeq ($(os),Darwin)
# pkgdeps := #name of the pkgs on brew osx
# endif

# ifeq ($(os),MinGW)
# pkgdeps := #name of the pkgs on mingw pacman : eq : mingw-w64-x86_64-xxxx
# endif
