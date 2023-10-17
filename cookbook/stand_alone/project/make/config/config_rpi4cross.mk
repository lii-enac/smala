build_dir ?= build

djnn-pkgconf :=
smala-pkgconf :=
djnn_cpp_path ?= ../djnn-cpp
smala_path ?= ../smala

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

no_pch := yes

#os = Linux
compiler := llvm
linker := gnu
rpi_sysroot := /Users/conversy/recherche/istar/code/misc/rpi_home/sysroot
isa_os_arch := arm-linux-gnueabihf
cc_version := 10

CC     := `brew --prefix llvm`/bin/clang
CXX    := `brew --prefix llvm`/bin/clang++
CXXFLAGS    += --target=$(isa_os_arch)
CXXFLAGS    += -fPIC
CXXFLAGS    += -Wno-c++20-designator
LDFLAGS    += --target=$(isa_os_arch)

export PKG_CONFIG_PATH=$(rpi_sysroot)/usr/lib/$(isa_os_arch)/pkgconfig

CXXFLAGS += \
	--sysroot=$(rpi_sysroot) \
    -isysroot $(rpi_sysroot) \
    -isystem $(rpi_sysroot)/usr/include/c++/$(cc_version) \
    -isystem $(rpi_sysroot)/usr/include/$(isa_os_arch)/c++/$(cc_version)

LDFLAGS += \
	--sysroot=$(rpi_sysroot) \
	-L$(rpi_sysroot)/usr/lib/gcc/$(isa_os_arch)/$(cc_version) \
    -B$(rpi_sysroot)/usr/lib/gcc/$(isa_os_arch)/$(cc_version)

LIBS += \
	-Wl,--no-undefined \
	-Wl,-rpath-link,$(rpi_sysroot)/lib/$(isa_os_arch):$(rpi_sysroot)/usr/lib/$(isa_os_arch) \
	-Wl,-rpath-link,$(abspath $(djnn_lib_path)):$(abspath $(build_dir))/lib \
	-Wl,-rpath,/root/djnn-cpp/build/lib:/root/smala/build/lib


