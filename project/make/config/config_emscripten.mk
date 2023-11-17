build_dir ?= build

djnn-pkgconf :=
djnn_cpp_path ?= ../djnn-cpp

no_pch := yes

# very fast compilation, but no debug
CXXFLAGS += -O0
# fast compilation and debug
# CXXFLAGS = -O2 -g
# optimized but slower compilation
# CXXFLAGS += -O3
# optimized and debug but even slower compilation
# CXXFLAGS += -O3 -g

CXXFLAGS += -g
CXXFLAGS += -ftime-trace

#CXXFLAGS += -fsanitize=address
#LDFLAGS += -fsanitize=address

# CXXFLAGS += -fsanitize=thread -O1
# LDFLAGS += -fsanitize=thread


# --
# cross-compile emscripten

use_pch := no

os := em
compiler := llvm
linker := llvm

CC_CK  := `brew --prefix emscripten`/bin/emcc
CXX_CK := `brew --prefix emscripten`/bin/em++
AR := `brew --prefix emscripten`/bin/emar
CFLAGS_CK += -fPIC
CXXFLAGS_CK += -Wno-c++20-designator

DYNLIB = -r #-shared
lib_suffix=.bc
EXE := .html
launch_cmd := emrun --browser safari

EMFLAGS := \
-s USE_SDL=2 \
-pthread \
-gsource-map

em_ext_libs_path ?= ../../misc/emscripten/local-install

EMCFLAGS += $(EMFLAGS)
EMCFLAGS += -I$(em_ext_libs_path)/include -I$(shell brew --prefix)/opt/flex/include -I/usr/local/include #glm

CFLAGS_CK += $(EMCFLAGS)
CFLAGS_CK += -DGL2D_GL_CONTEXT_PROFILE_ES=1

LDFLAGS_CK += $(EMFLAGS) \
-s USE_SDL=2 \
-s FULL_ES3 \
-s MIN_WEBGL_VERSION=2 \
-s MAX_WEBGL_VERSION=2 \
-s ASSERTIONS=2 \
-s NO_DISABLE_EXCEPTION_CATCHING \
-s ERROR_ON_UNDEFINED_SYMBOLS=1 \
-s PTHREAD_POOL_SIZE=5

LDFLAGS_CK += -L$(em_ext_libs_path)/lib

#export PKG_CONFIG_PATH=$(rpi_sysroot)/usr/lib/$(isa_os_arch)/pkgconfig

# lib_soname_ += \
# 	-Wl,-rpath-link,$(rpi_sysroot)/lib/$(isa_os_arch):$(rpi_sysroot)/usr/lib/$(isa_os_arch) \
# 	-Wl,-rpath,$(install_dir)/djnn-cpp/build/lib

YACC := `brew --prefix`/opt/bison/bin/bison -d -Wno-conflicts-sr -Wno-conflicts-rr
LEX := `brew --prefix`/opt/flex/bin/flex

