ifndef os
os := $(shell uname -s)

ifeq ($(findstring MINGW,$(os)),MINGW)
os := MinGW
endif
endif

ifeq ($(os),Linux)
compiler ?= gnu
CFLAGS_COMMON +=  -fpic
YACC = bison -d -W
LD_LIBRARY_PATH=LD_LIBRARY_PATH
debugger := gdb
lib_suffix =.so
DYNLIB = -shared
LDFLAGS_SC += -lstdc++fs
endif

ifeq ($(os),Darwin)
compiler ?= llvm
ifeq ($(PREFIX),)
brew_prefix := $(shell brew --prefix)
else
brew_prefix := $(HOMEBREW_PREFIX)
endif
YACC := $(brew_prefix)/opt/bison/bin/bison -d
LEX := $(brew_prefix)/opt/flex/bin/flex
LD_LIBRARY_PATH=DYLD_LIBRARY_PATH
# https://stackoverflow.com/a/33589760
debugger := PATH=/usr/bin /Applications/Xcode.app/Contents/Developer/usr/bin/lldb
#other_runtime_lib_path := /Users/conversy/src-ext/SwiftShader/build
other_runtime_lib_path := /Users/conversy/recherche/istar/code/misc/MGL/build
CXXFLAGS_SC += -I$(shell brew --prefix flex)/include
LDFLAGS_SC += -L$(shell brew --prefix flex)/lib
lib_suffix =.dylib
DYNLIB = -dynamiclib
endif

ifeq ($(os),MinGW)
compiler ?= gnu
CFLAGS_COMMON += -fpic
YACC = bison -d -W
LD_LIBRARY_PATH=PATH
debugger := gdb
lib_suffix =.dll
DYNLIB = -shared
endif
