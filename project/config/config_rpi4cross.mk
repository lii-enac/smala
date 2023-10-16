build_dir := build


#devel
#djnn-pkgconf = djnn-cpp-dev
#install (github)
djnn-pkgconf = djnn-cpp

#or use on local
djnn_path = $(abspath ../djnn-cpp)


cookbook_app_for_make_test := button
cookbook_apps_extra :=

#smala flags
SMAFLAGS += -g
#SMAFLAGS += -gen-cleaner
#SMAFLAGS += -fastcomp
djnn_modules ?= animation comms audio gui display input files utils base exec_env core
#djnn_modules += c_api

#C++ flags 
CFLAGS_COMMON += -g #-fstandalone-debug -fno-limit-debug-info
#LDFLAGS_COMMON +=

#Sanitizer
#CFLAGS_COMMON += -fsanitize=address
#LDFLAGS_COMMON += -fsanitize=address
#CFLAGS_CK += -fsanitize=thread
#LDFLAGS_CK += -fsanitize=thread
#CFLAGS_COMMON += -fsanitize=memory
#LDFLAGS_COMMON += -fsanitize=memory

#CXXFLAGS_CK += -DDJNN_STL_DJNN=1
CXXFLAGS_CK += -O0
#CXXFLAGS_CK += -ftime-trace

#use_pch := no

# cross-compile support
#cross_prefix := em
#cookbook_cross_prefix := em

# emscripten ext libs
#em_ext_libs_path := ../djnn-emscripten-ext-libs


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
