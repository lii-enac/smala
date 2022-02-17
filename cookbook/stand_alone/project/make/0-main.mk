# canonical Makefile for smala applications
# 1. copy stand_alone directory somwhere: cp -r cookbook/stand_alone /some/where/else
# 2. edit configuration part (executable name, srcs, djnn_libs, path to djnn-cpp and smalac)
# 3. make test



#---- pkg-config -- 
# install made with 
# 	- make install in djnn-cpp and smala
#   - or with package system : brew, apt, pacman ... 

ifneq ($(djnn-pkgconf),)
djnn_cflags := $(shell pkg-config $(djnn_pkgconf) --cflags)
djnn_ldflags := $(shell pkg-config $(djnn_pkgconf) --libs)
djnn_lib_path := $(shell pkg-config $(djnn_pkgconf) --libs-only-L)
djnn_lib_path := $(subst -L, , $(djnn_lib_path))
endif

ifneq ($(smala-pkgconf),)
#smalac := "should be in /usr/(local)/bin"
smala_cflags := $(shell pkg-config $(smala_pkgconf) --cflags)
smala_ldflags := $(shell pkg-config $(smala_pkgconf) --libs)
smala_lib_path := $(shell pkg-config $(smala_pkgconf) --libs-only-L)
smala_lib_path := $(subst -L, , $(smala_lib_path))
endif


#---- local sources



ifneq ($(djnn_cpp_path),)
djnn_cflags := -I$(djnn_cpp_path)/src -I$(djnn_cpp_path)/src
#djnn_cflags += -I$(smala_lib_dir)
djnn_lib_path := $(djnn_cpp_path)/build/lib
djnn_ldflags := -L$(djnn_lib_path) -ldjnn-core -ldjnn-base -ldjnn-animation -ldjnn-audio \
				-ldjnn-comms -ldjnn-display -ldjnn-exec_env -ldjnn-files -ldjnn-gui \
				-ldjnn-input -ldjnn-utils
endif
#djnn_ldflags += -L$(smala_lib_dir) -lsmala

ifneq ($(smala_path),)
smalac := $(smala_path)/build/smalac
smala_cflags := -I$(smala_path)/build/src_lib
smala_ldflags := -L$(smala_path)/build/lib -lsmala
smala_lib_path := $(smala_path)/build/lib
smala_lib_dir ?= $(smala_path)/build/lib
endif

# for emscripten
em_ext_libs_path := ../../../djnn-emscripten-ext-libs

# -------------------------------------------------------------------
# hopefully no need to tweak the lines below

# remove builtin rules: speed up build process and help debug
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:

ifndef os
os := $(shell uname -s)

ifeq ($(findstring MINGW,$(os)),MINGW)
os := MinGW
endif
endif

# ---------------------------------------
# save user-provided CXXFLAGS, and use CXXFLAGS as the utltimate compiler configuration

CXXFLAGS_CFG := $(CXXFLAGS)
CXXFLAGS :=

# cross-compile support
ifndef cross_prefix
cross_prefix := g
#cross_prefix := em
#options: g llvm-g i686-w64-mingw32- arm-none-eabi- em
#/Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-c
#/usr/local/Cellar/android-ndk/r14/toolchains/arm-linux-androideabi-4.9/prebuilt/darwin-x86_64/bin/arm-linux-androideabi-g
endif

CC := $(cross_prefix)cc
CXX := $(cross_prefix)++

ifeq ($(cross_prefix),em)
os := em
EXE := .html
launch_cmd := emrun

EMFLAGS := -Wall -Wno-unused-variable -Oz \
-s USE_BOOST_HEADERS -s USE_SDL=2 -s USE_SDL_IMAGE=2 -s USE_FREETYPE=1 -s USE_WEBGL2=1 \
-DSDL_DISABLE_IMMINTRIN_H \
-s EXPORT_ALL=1 -s DISABLE_EXCEPTION_CATCHING=0 \
-s DISABLE_DEPRECATED_FIND_EVENT_TARGET_BEHAVIOR=1 \
-s ASSERTIONS=2 \
-s ERROR_ON_UNDEFINED_SYMBOLS=0

em_ext_libs_path ?= ../djnn-emscripten-ext-libs

#idn2 expat curl fontconfig unistring psl 
ext_libs := expat curl
ext_libs := $(addprefix $(em_ext_libs_path)/lib/lib,$(addsuffix .a, $(ext_libs))) -lopenal

EMCFLAGS += $(EMFLAGS) -I$(em_ext_libs_path)/include -I/usr/local/include #glm
CFLAGS += $(EMCFLAGS)
CXXFLAGS += $(EMCFLAGS)
LDFLAGS += $(EMFLAGS) \
	$(ext_libs) \
	--emrun \
	--preload-file $(res_dir)@$(res_dir) \
	--preload-file /Library/Fonts/Arial.ttf@/usr/share/fonts/Arial.ttf

endif

CXXFLAGS_COMMON += -MMD -g -std=c++14

ifeq ($(os),Linux)
compiler ?= gnu
LD_LIBRARY_PATH=LD_LIBRARY_PATH
debugger := gdb
endif

ifeq ($(os),Darwin)
compiler ?= llvm
LD_LIBRARY_PATH=DYLD_LIBRARY_PATH
# https://stackoverflow.com/a/33589760
debugger := PATH=/usr/bin /Applications/Xcode.app/Contents/Developer/usr/bin/lldb
endif

ifeq ($(os),MinGW)
compiler ?= gnu
LD_LIBRARY_PATH=PATH
debugger := gdb
endif

ifeq ($(os),em)
compiler ?= llvm
LD_LIBRARY_PATH=LD_LIBRARY_PATH
debugger := gdb
EXE := .html
endif

exe := $(exe)$(EXE)
exe := $(build_dir)/$(exe)

default: $(exe)
.PHONY: default

test: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)="$(abspath $(djnn_lib_path))":"$(abspath $(smala_lib_path))":$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)")
dbg: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)="$(abspath $(djnn_lib_path))":"$(abspath $(smala_lib_path))":$$$(LD_LIBRARY_PATH) $(debugger) "$(shell pwd)/$(exe)")
.PHONY: test

LD  = $(CXX)

objs_sma := $(srcs_sma:.sma=.o)
objs_sma := $(addprefix $(build_dir)/,$(objs_sma))
objs_other := $(srcs_other:.cpp=.o)
objs_other := $(addprefix $(build_dir)/,$(objs_other))

objs := $(objs_sma) $(objs_other)

gensrcs := $(objs_sma:.o=.cpp)
#$(objs_sma): $(gensrcs) # this forces the right language to compile the generated sources, but it will rebuild all sma files


ifeq ($(cross_prefix),em)
app_libs := $(addsuffix .bc,$(addprefix $(djnn_lib_path)/libdjnn-,$(djnn_libs)))
else
app_libs := $(addprefix -ldjnn-,$(djnn_libs))
endif

# ---------------------------------------
# precompiled headers

# https://stackoverflow.com/questions/58841/precompiled-headers-with-gcc
# https://stackoverflow.com/questions/26755219/how-to-use-pch-with-clang

ifeq ($(compiler),llvm)
pch_ext = .pch
endif
ifeq ($(compiler),gnu)
pch_ext = .gch
endif

pch_file := precompiled.h
pch_src := src/$(pch_file)
pch_dst := $(build_dir)/$(pch_src)$(pch_ext)

# SDL and other stuff define new variables for compiling, canceling the use of pch with gnu cc
# FIXME this is not safe as every other external lib may define something
ifeq ($(compiler),gnu)
# https://gitlab.gnome.org/GNOME/gnome-online-accounts/-/merge_requests/14
# Both GCC and Clang appear to expand -pthread to define _REENTRANT on their own
CXXFLAGS_PCH_DEF += -D_REENTRANT
ifeq ($(display),SDL)
CXXFLAGS_PCH_DEF += -Dmain=SDL_main
endif
endif

$(pch_dst): $(pch_src)
	@mkdir -p $(dir $@)
ifeq ($V,max)
	$(CXX) -x c++-header $(CXXFLAGS) $< -o $@
else
	@$(call rule_message,compiling,$(stylized_target))
	$(CXX) -x c++-header $(CXXFLAGS) $< -o $@
endif

ifeq ($(compiler),llvm)
CXXFLAGS_PCH_INC += -include-pch $(pch_dst)
#-fpch-instantiate-templates -fpch-codegen -fpch-debuginfo
endif
ifeq ($(compiler),gnu)
# https://stackoverflow.com/a/3164874
CXXFLAGS_PCH_INC += -I$(dir $(pch_dst)) -include $(pch_file) -Winvalid-pch
#-fno-implicit-templates
#$(build_dir)/src/core/utils/build/external_template.o: CXXFLAGS += -fimplicit-templates
endif

$(pch_dst): override CXXFLAGS = $(CXXFLAGS_CFG) $(CXXFLAGS_PCH_DEF) $(djnn_cflags) $(CXXFLAGS_COMMON) $(CXXFLAGS_CK)

pch: $(pch_dst)
clean_pch:
	rm -f $(pch_dst)

$(objs): $(pch_dst)

#----------------------------

#$(objs): CXXFLAGS += $(djnn_cflags) $(smala_cflags) -I$(src_dir) -I$(build_dir)/$(src_dir) -I$(build_dir)/lib

$(objs): CXXFLAGS = $(CXXFLAGS_CFG) $(CXXFLAGS_PCH_DEF) $(CXXFLAGS_PCH_INC) $(djnn_cflags) $(smala_cflags) -I$(src_dir) -I$(build_dir)/$(src_dir) -I$(build_dir)/lib\
	$(CXXFLAGS_COMMON) $(CXXFLAGS_CK)

$(exe): LDFLAGS += $(djnn_ldflags) $(smala_ldflags)
$(exe): LIBS += $(app_libs)

$(exe): $(objs)
	@mkdir -p $(dir $@)
	$(LD) $^ -o $@ $(LDFLAGS) $(LIBS)

# .sma to .cpp, .c etc
$(build_dir)/%.cpp $(build_dir)/%.h: %.sma
	@mkdir -p $(dir $@)
	@echo smalac -cpp $^ -builddir $(dir $@)
	@$(smalac) -cpp $^ -builddir $(dir $@)

# from .c user sources
$(build_dir)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# from .cpp user sources
$(build_dir)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# for .c generated sources
$(build_dir)/%.o: $(build_dir)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# for .cpp generated sources
$(build_dir)/%.o: $(build_dir)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@


deps := $(objs:.o=.d)
-include $(deps)

include $(project_dir)/9-pkgdeps.mk

# --

distclean clear:
	rm -rf build
clean:
	rm -f $(gensrcs) $(objs) $(deps)
.PHONY: clean clear distclean

foo:
	echo $(objs_other)

