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
# cross-compile rpi4 debian

use_pch := no

#os := Linux
compiler := llvm
linker := gnu
rpi_sysroot := /Users/conversy/recherche/istar/code/misc/rpi_home/sysroot
isa_os_arch := arm-linux-gnueabihf
cc_version := 10

CC_CK  := `brew --prefix llvm`/bin/clang
CXX_CK := `brew --prefix llvm`/bin/clang++
CXXFLAGS_CK += --target=$(isa_os_arch)
CXXFLAGS_CK += -fPIC
CXXFLAGS_CK += -Wno-c++20-designator
LDFLAGS_CK += --target=$(isa_os_arch)

DYNLIB := -shared
lib_suffix =.so


export PKG_CONFIG_PATH=$(rpi_sysroot)/usr/lib/$(isa_os_arch)/pkgconfig

CXXFLAGS_CK += \
	--sysroot=$(rpi_sysroot) \
    -isysroot $(rpi_sysroot) \
    -isystem $(rpi_sysroot)/usr/include/c++/$(cc_version) \
    -isystem $(rpi_sysroot)/usr/include/$(isa_os_arch)/c++/$(cc_version)

LDFLAGS_CK += \
	--sysroot=$(rpi_sysroot) \
	-L$(rpi_sysroot)/usr/lib/gcc/$(isa_os_arch)/$(cc_version) \
    -B$(rpi_sysroot)/usr/lib/gcc/$(isa_os_arch)/$(cc_version)

LDFLAGS_CK += \
	-Wl,--no-undefined \
	-Wl,-rpath-link,$(rpi_sysroot)/lib/$(isa_os_arch):$(rpi_sysroot)/usr/lib/$(isa_os_arch) \
	-Wl,-rpath-link,$(abspath $(djnn_lib_path)):$(abspath $(build_dir))/lib \
	-Wl,-rpath,/root/djnn-cpp/build/lib:/root/smala/build/lib


YACC := `brew --prefix`/opt/bison/bin/bison -d -Wno-conflicts-sr -Wno-conflicts-rr
LEX := `brew --prefix`/opt/flex/bin/flex
