#	djnn smalac compiler
#
#	The copyright holders for the contents of this file are:
#		Ecole Nationale de l'Aviation Civile, France (2017-2020)
#	See file "license.terms" for the rights and conditions
#	defined by copyright holders.
#
#
#	Contributors:
#		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
#		Stéphane Conversy <stephane.conversy@enac.fr>
#		Mathieu Poirier	  <mathieu.poirier@enac.fr>
#

default: config.mk smalac
.PHONY: default

all: config.mk smalac smala_lib cookbook_apps test_apps
.PHONY: all

help:
	@echo "default: smalac ; all: smalac cookbook"
	@echo "button: will build button cookbook app (works with all cookbook apps: $(cookbook_apps))"
	@echo "button_test: will build button cookbook app and launch it (works with all cookbook apps: $(cookbook_apps))"
	@echo "experiment make -j !!"

config.mk:
	cp config.default.mk config.mk
include config.default.mk
-include config.mk

MAJOR = 1
MINOR = 14
MINOR2 = 0

# remove builtin rules : speed up build process and help debug
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:

# ---------------------------------------
# utils

# https://stackoverflow.com/a/16151140
uniq = $(if $1,$(firstword $1) $(call uniq,$(filter-out $(firstword $1),$1)))

# recursive wildcard https://stackoverflow.com/a/12959694
rwildcard = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
rwildcardmul = $(wildcard $(addsuffix $2, $1)) $(foreach d,$(wildcard $(addsuffix *, $1)),$(call rwildcard,$d/,$2))

# ---------------------------------------
# os

ifndef os
os := $(shell uname -s)

ifeq ($(findstring MINGW,$(os)),MINGW)
os := MinGW
endif
endif

# ---------------------------------------
# cross-compile support

ifdef cross_prefix
#cross_prefix := c
#options: c g llvm-g i686-w64-mingw32- arm-none-eabi- em
#/Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-c
#/usr/local/Cellar/android-ndk/r14/toolchains/arm-linux-androideabi-4.9/prebuilt/darwin-x86_64/bin/arm-linux-androideabi-g
ifneq ($(cross_prefix),g)
CC := $(cross_prefix)$(CC)
CXX := $(cross_prefix)$(CXX)
AR := $(cross_prefix)$(AR)
RANLIB := $(cross_prefix)$(RANLIB)
SIZE := $(cross_prefix)$(SIZE)
else
#temporary covid hack
CC := gcc
CXX := g++
#AR := $(cross_prefix)$(AR)
RANLIB := ranlib
SIZE ?=
endif
else
CXX_SC := $(CXX)
endif

ifdef cookbook_cross_prefix
#cross_prefix := c
#options: c g llvm-g i686-w64-mingw32- arm-none-eabi- em
#/Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-c
#/usr/local/Cellar/android-ndk/r14/toolchains/arm-linux-androideabi-4.9/prebuilt/darwin-x86_64/bin/arm-linux-androideabi-g
ifneq ($(cookbook_cross_prefix),g)
CC_CK := $(cookbook_cross_prefix)$(CC)
CXX_CK ?= $(cookbook_cross_prefix)$(CXX)
AR_CK := $(cookbook_cross_prefix)$(AR)
RANLIB_CK := $(cookbook_cross_prefix)$(RANLIB)
SIZE_CK := $(cookbook_cross_prefix)$(SIZE)
else
#temporary covid hack
CC_CK := gcc
CXX_CK := g++
#AR := $(cross_prefix)$(AR)
RANLIB_CK := ranlib
SIZE_CK ?=
endif
else
CC_CK := $(CC)
CXX_CK := $(CXX)
endif

# # cross-compile support
# ifndef cross_prefix
# ifeq ($(os),Darwin)
# cross_prefix := llvm-g
# endif
# #ifeq ($(os),Linux)
# cross_prefix := g
# #endif
# #cross_prefix := arm-none-eabi-
# #cross_prefix := em
# #cross_prefix := i686-w64-mingw32-
# endif

# ifndef cookbook_cross_prefix
# ifeq ($(os),Darwin)
# cookbook_cross_prefix := llvm-g
# endif
# #ifeq ($(os),Linux)
# cookbook_cross_prefix := g
# #endif
# #cross_prefix := arm-none-eabi-
# #cross_prefix := em
# #cross_prefix := i686-w64-mingw32-
# endif

lib_smala_name = libsmala

#CC := $(cross_prefix)cc
#CXX := $(cross_prefix)++
#CC_CK := $(cookbook_cross_prefix)cc
#CXX_CK := $(cookbook_cross_prefix)++
CFLAGS += -g -MMD
CXXFLAGS += $(CFLAGS) -std=c++17
#LIBS ?=

install_brew : djnn_path =
install_brew : install

ifeq ($(djnn_path),) 
djnn_cflags := $(shell pkg-config $(djnn-pkgconf) --cflags)
djnn_ldflags := $(shell pkg-config $(djnn-pkgconf) --libs-only-L)
djnn_ldlibs := $(shell pkg-config $(djnn-pkgconf) --libs-only-l)
djnn_libs := $(shell pkg-config $(djnn-pkgconf) --libs)
djnn_lib_path := $(shell pkg-config $(djnn-pkgconf) --libs-only-L)
djnn_lib_path := $(subst -L, , $(djnn_lib_path))
else
djnn_cflags := -I$(djnn_path)/src
djnn_ldflags := -L$(djnn_path)/build/lib
#djnn_ldlibs := -ldjnn-core -ldjnn-base -ldjnn-animation -ldjnn-audio -ldjnn-comms -ldjnn-display -ldjnn-exec_env -ldjnn-files -ldjnn-gui -ldjnn-input -ldjnn-utils
djnn_ldlibs := -ldjnn-animation -ldjnn-comms -ldjnn-gui  -ldjnn-display -ldjnn-input -ldjnn-files -ldjnn-utils -ldjnn-base -ldjnn-exec_env -ldjnn-core 
djnn_libs := $(djnn_ldflags) $(djnn_ldlibs)
djnn_lib_path := $(djnn_path)/build/lib
endif

# for filesystem.h
CXXFLAGS_SC += $(djnn_cflags)

ifeq ($(os),Linux)
#CXXFLAGS +=
CFLAGS +=  -fpic
YACC = bison -d -W
LD_LIBRARY_PATH=LD_LIBRARY_PATH
debugger := gdb
lib_suffix =.so
DYNLIB = -shared
LDFLAGS_SC += -lstdc++fs
endif

ifeq ($(os),Darwin)
#CFLAGS +=
YACC = /usr/local/opt/bison/bin/bison -d -W #--report=state
LEX = /usr/local/opt/flex/bin/flex
LD_LIBRARY_PATH=DYLD_LIBRARY_PATH
# https://stackoverflow.com/a/33589760
debugger := PATH=/usr/bin /Applications/Xcode.app/Contents/Developer/usr/bin/lldb
#other_runtime_lib_path := /Users/conversy/src-ext/SwiftShader/build
CXXFLAGS_SC += -I/usr/local/opt/flex/include
LDFLAGS_SC += -L/usr/local/opt/flex/lib
lib_suffix =.dylib
DYNLIB = -dynamiclib
endif

ifeq ($(os),MinGW)
CFLAGS += -fpic
YACC = bison -d -W
LD_LIBRARY_PATH=PATH
debugger := gdb
lib_suffix =.dll
DYNLIB = -shared
endif


ifeq ($(cookbook_cross_prefix),arm-none-eabi-)
CXXFLAGS_CK += -Wno-psabi #https://stackoverflow.com/a/48149400
endif

ifeq ($(cookbook_cross_prefix),em)
#os := em
EXE := .html
launch_cmd := emrun --serve_after_close

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
CFLAGS_CK = $(EMCFLAGS)
CXXFLAGS_CK += $(EMCFLAGS)
LDFLAGS_CK += $(EMFLAGS) \
	$(ext_libs) \
	--emrun
endif

YACC ?= bison
LEX ?= flex


#CFLAGS += -fsanitize=thread -O1
#LDFLAGS += -fsanitize=thread

#CFLAGS += -fsanitize=address -O1
#LDFLAGS += -fsanitize=address

#CFLAGS += -fsanitize=memory -O1
#LDFLAGS += -fsanitize=memory

# -----------
# smalac

bin_name := smalac

smalac_objs := parser.o scanner.o type_manager.o cpp_type_manager.o argument.o driver.o node.o smala_native.o ctrl_node.o \
	newvar_node.o instruction_node.o binary_instruction_node.o \
	native_expression_node.o native_component_node.o range_node.o set_parent_node.o transition_node.o preamble.o ast.o \
	builder.o cpp_builder.o main.o parser.o scanner.o

smalac_objs := $(addprefix $(build_dir)/src/, $(smalac_objs))

smalac := $(build_dir)/$(bin_name)

smalac: config.mk $(smalac)
.PHONY: smalac

$(smalac): $(smalac_objs)
	$(CXX) $^ -o $@ $(LDFLAGS)

$(smalac): CFLAGS += $(CXXFLAGS_SC) -Isrc -I$(build_dir)/src -I$(build_dir)/lib
$(smalac): CXX = $(CXX_SC)
$(smalac): LDFLAGS += $(LDFLAGS_SC)

# ------------
# smala lib

smala_lib_dir := src_lib
smala_lib := $(build_dir)/lib/$(lib_smala_name)$(lib_suffix)
smala_lib_srcs := $(shell find $(smala_lib_dir) -name "*.sma")
smala_lib_objs := $(addprefix $(build_dir)/, $(patsubst %.sma,%.o,$(smala_lib_srcs)))
smala_lib_headers := $(addprefix $(build_dir)/, $(patsubst %.sma,%.h,$(smala_lib_srcs)))

$(smala_lib_objs): CFLAGS += $(djnn_cflags)
$(smala_lib_objs): CXX = $(CXX_CK)

$(smala_lib): $(smala_lib_objs) 
	@mkdir -p $(dir $@)
	$(CXX_CK) $(DYNLIB) -o $@ $^ $(LDFLAGS_CK) $(djnn_libs)

smala_lib: $(smala_lib)
.PRECIOUS: $(smala_lib_headers)

lib: smala_lib

# ------------
# automatic rules

$(build_dir)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# for generated .cpp
$(build_dir)/%.o: $(build_dir)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(build_dir)/%.cpp $(build_dir)/%.hpp: %.y
	@mkdir -p $(dir $@)
	$(YACC) -v -o $@ $<

$(build_dir)/%.cpp: %.l
	@mkdir -p $(dir $@)
	$(LEX) -o $@ $<


# ------------
# specific dependencies and peculiarities

$(build_dir)/src/scanner.o: CXXFLAGS += -Dregister=""
$(build_dir)/src/location.hh: $(build_dir)/src/parser.cpp

# for initial make -j
# find build -name "*.d" | xargs grep -s "parser.hpp" | awk '{print $1}' | awk -F "." '{print $1".o"}' | sed s/build/\$\(build_dir\)/ | xargs echo
$(build_dir)/src/scanner.o $(build_dir)/src/ast.o $(build_dir)/src/j_builder.o $(build_dir)/src/builder.o \
$(build_dir)/src/main.o $(build_dir)/src/cpp_builder.o $(build_dir)/src/c_builder.o $(build_dir)/src/parser.o $(build_dir)/src/driver.o: $(build_dir)/src/parser.hpp

$(build_dir)/src/scanner.o $(build_dir)/src/ast.o $(build_dir)/src/newvar_node.o $(build_dir)/src/range_node.o\
$(build_dir)/src/native_expression_node.o $(build_dir)/src/native_component_node.o $(build_dir)/src/builder.o \
$(build_dir)/src/smala_native.o $(build_dir)/src/cpp_builder.o $(build_dir)/src/preamble.o $(build_dir)/src/instruction_node.o \
$(build_dir)/src/set_parent_node.o $(build_dir)/src/main.o $(build_dir)/src/binary_instruction_node.o \
$(build_dir)/src/parser.o $(build_dir)/src/driver.o $(build_dir)/src/node.o $(build_dir)/src/ctrl_node.o $(build_dir)/src/transition_node.o: $(build_dir)/src/location.hh


# ------------
# merr error reporting tool for smalac

merr := $(build_dir)/merr/merr
merr_objs := merr.o
merr_objs := $(addprefix $(build_dir)/merr/, $(merr_objs))

merr_errs := merr/smalac_errors.merr

merr: $(merr) $(merr_errs)
	$(MAKE) clean_merr
	$(MAKE) smalac
	$(merr) $(merr_errs) -o src/errors.h -c $(smalac)
	$(MAKE) smalac
.PHONY: merr

clean_merr:
	rm -f src/errors.h
	touch src/errors.h
.PHONY: clean_merr

$(merr_objs): CXXFLAGS += -std=c++1z 

$(merr): $(merr_objs)
	$(CXX) $^ -o $@

$(build_dir)/src/parser.cpp: src/errors.h

# -----------
# cookbook apps

define cookbookapp_makerule
djnn_libs_cookbook_app :=
smala_libs_cookbook_app :=
pkg_cookbook_app :=
libs_cookbook_app :=
cflags_cookbook_app :=
cppflags_cookbook_app :=
res_dir :=
other_runtime_lib_path :=

ckappname := $$(notdir $1)
$1_app_srcs_dir := cookbook/$1
app_srcs_dir := $$($1_app_srcs_dir)
gen_srcs_dir := $(build_dir)/cookbook/$1

include cookbook/$1/cookbook_app.mk

$1_app_objs := $$(objs_cookbook_app)
$1_app_gensrcs := $$($1_app_objs:.o=.cpp)
$1_app_gensrcs := $$(addprefix $(build_dir)/cookbook/$1/, $$($1_app_gensrcs))
$1_app_objs := $$(addprefix $(build_dir)/cookbook/$1/, $$($1_app_objs))
$1_app_exe := $$(build_dir)/cookbook/$1/$$(ckappname)_app$$(EXE)
$1_res_dir := $$(res_dir)
$1_app_cppflags := $$(cppflags_cookbook_app)
$1_app_cflags := $$(cflags_cookbook_app)
$1_app_pkg := $$(pkg_cookbook_app)
$1_other_runtime_lib_path := $$(other_runtime_lib_path)

ifeq ($$(cookbook_cross_prefix),em)
$1_app_libs := $$(addsuffix .bc,$$(addprefix $$(djnn_lib_path)/libdjnn-,$$(djnn_libs_cookbook_app))) $$(libs_cookbook_app) \
	--preload-file $$($1_app_srcs_dir)/$$($1_res_dir)@$$($1_res_dir) \
	--preload-file /Library/Fonts/Arial.ttf@/usr/share/fonts/Arial.ttf
else
$1_app_libs := $$(addprefix -ldjnn-,$$(djnn_libs_cookbook_app)) $$(libs_cookbook_app)
ifneq ($$(smala_libs_cookbook_app),)
$1_app_cppflags += -I$$(build_dir)/$(smala_lib_dir)
$1_app_libs := -Lbuild/lib $$(addprefix -l,$$(smala_libs_cookbook_app)) $$(call uniq,$$(djnn_libs) $$($1_app_libs)) #$(djnn_libs) is necessary for linux's ld
$$($1_app_objs): $$(smala_lib)

$$(notdir $1)_toto:
	echo $$(smala_lib)
endif

ifneq ($$($1_app_pkg),)
#$1_lib_pkgpath = $$(subst $$() $$(),:,$$(lib_pkgpath))
$1_app_cppflags += $$(shell env PKG_CONFIG_PATH=$$(PKG_CONFIG_PATH):$$($1_lib_pkgpath) pkg-config --cflags $$($1_app_pkg))
$1_app_libs += $$(shell env PKG_CONFIG_PATH=$$(PKG_CONFIG_PATH):$$($1_lib_pkgpath) pkg-config --libs $$($1_app_pkg))
endif

endif

$1_app_link := $$(CXX_CK)

#$$($1_app_objs): $$($1_app_gensrcs)
$$($1_app_objs): CC = $$(CC_CK)
$$($1_app_objs): CXX = $$(CXX_CK)
$$($1_app_objs): CFLAGS += $$(djnn_cflags) $$(CXXFLAGS_CK) $$($1_app_cppflags) $$($1_app_cflags)
$$($1_app_exe): LDFLAGS_CK += $$(djnn_ldflags)
$$($1_app_exe): LIBS += $$($1_app_libs)

$$($1_app_exe): $$($1_app_objs)
	$$($1_app_link) $$^ -o $$@ $$(LDFLAGS_CK) $$(LIBS)

$$(notdir $1)_objs: $$($1_app_objs)

$$(notdir $1): $$($1_app_exe)

$$(notdir $1)_test: $$(notdir $1)
	(cd "$$($1_app_srcs_dir)"; env $$(LD_LIBRARY_PATH)="$$($$(LD_LIBRARY_PATH)):$$(abspath $$(djnn_lib_path)):$$(abspath $$(build_dir)/lib):$$($1_other_runtime_lib_path)" $$(launch_cmd) "$$(shell pwd)/$$($1_app_exe)")
$$(notdir $1)_dbg: $$(notdir $1)
	(cd "$$($1_app_srcs_dir)"; env $$(LD_LIBRARY_PATH)="$$($$(LD_LIBRARY_PATH)):$$(abspath $$(djnn_lib_path)):$$(abspath $$(build_dir)/lib):$$($1_other_runtime_lib_path)" $$(debugger) "$$(shell pwd)/$$($1_app_exe)")

$$(notdir $1)_clean:
	rm -f $$($1_app_exe) $$($1_app_objs) $$($1_app_gensrcs)
$$(notdir $1)_clear:
	rm -rf $(build_dir)/cookbook/$1

.PHONY: $1 $1_test $1_dbg

$$(notdir $1)_dbg_print:
	@echo $1_dbg
	@echo $$($1_app_objs)
	@echo $$($1_app_gensrcs)

deps += $$($1_app_objs:.o=.d)
endef


cookbook_apps := $(shell cd cookbook && find * -name cookbook_app.mk | xargs -I{} dirname {})
disable_cookbook_apps ?= \
	extra/crazyflie \
	extra/crazyflie_drone_app \
	comms/swim \
	comms/midi \
	gui/physics

cookbook_apps := $(filter-out $(disable_cookbook_apps),$(cookbook_apps))

cookbook_apps += $(cookbook_apps_extra)

$(foreach a,$(cookbook_apps),$(eval $(call cookbookapp_makerule,$a)))

cookbook_apps: $(notdir $(cookbook_apps))
cookbook_apps_test: $(addsuffix _test,$(notdir $(cookbook_apps)))


# ---------------------------------------
# rules

# .sma to .cpp
$(build_dir)/%.cpp $(build_dir)/%.h: %.sma | $(smalac)
	@mkdir -p $(dir $@)
	@echo smalac $<
	@$(smalac) -g $< || (c=$$?; rm -f $*.cpp $*.h; (exit $$c))
	@mv $*.cpp $(build_dir)/$(*D)
	@if [ -f $*.h ]; then mv $*.h $(build_dir)/$(*D); fi;

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


# ---------------------------------------
# package config

ifeq ($(PREFIX),)
# dev install
smala_install_prefix :=  $(abspath $(build_dir))
pkg_config_install_prefix := /usr/local
else
# pkg install (brew, deb, arch)
smala_install_prefix := $(abspath $(DESTDIR)$(PREFIX))
pkg_config_install_prefix := $(abspath $(DESTDIR)$(PREFIX))
endif

pkgconfig_targets = smala-dev.pc smala.pc
pkgconfig_targets := $(addprefix $(build_dir)/, $(pkgconfig_targets))

pkgconf: $(pkgconfig_targets)

$(build_dir)/%.pc: distrib/%.pc.in
	@mkdir -p $(dir $@)
	@sed -e 's,@PREFIX@,$(smala_install_prefix),; s,@MAJOR@,$(MAJOR),; s,@MINOR@,$(MINOR),; s,@MINOR2@,$(MINOR2),' $< > $@


# -----------
# install

install_pkgconf: pkgconf
	test -d $(pkg_config_install_prefix)/lib/pkgconfig || mkdir -p $(pkg_config_install_prefix)/lib/pkgconfig	
ifeq ($(PREFIX),)
	install -m 644 $(build_dir)/smala-dev.pc	$(pkg_config_install_prefix)/lib/pkgconfig
else
	install -m 644 $(build_dir)/smala.pc	$(pkg_config_install_prefix)/lib/pkgconfig
endif

smala_lib_headers_no_src = $(patsubst $(build_dir)/src_lib/%,%,$(smala_lib_headers))

smala_libs_no_build_dir = $(patsubst $(build_dir)/lib/%,%,$(lib_smala_name)$(lib_suffix))

install_headers: $(addprefix $(smala_install_prefix)/include/smala/,$(smala_lib_headers_no_src))

install_bin: $(addprefix $(smala_install_prefix)/bin/,$(bin_name))

install_libs: $(addprefix $(smala_install_prefix)/lib/,$(smala_libs_no_build_dir))	

$(smala_install_prefix)/include/smala/%.h: $(build_dir)/src_lib/%.h
	@mkdir -p $(dir $@)
	install -m 644 $< $@

$(smala_install_prefix)/lib/%$(lib_suffix): $(build_dir)/lib/%$(lib_suffix)
	@mkdir -p $(dir $@)
ifneq ($(PREFIX),)
	install -m 644 $< $@
endif

$(smala_install_prefix)/bin/$(bin_name): build/$(bin_name)
	@mkdir -p $(dir $@)
	install -m 755 $< $@

install: default smala_lib install_pkgconf install_headers install_libs install_bin



# we have to redefine all variables already computed in config.mk
install_brew: djnn_cflags = $(shell pkg-config $(djnn-pkgconf) --cflags)
install_brew: djnn_ldflags = $(shell pkg-config $(djnn-pkgconf) --libs-only-L)
install_brew: djnn_ldlibs = $(shell pkg-config $(djnn-pkgconf) --libs-only-l)
install_brew: djnn_libs = $(shell pkg-config $(djnn-pkgconf) --libs)
install_brew: djnn_lib_path = $(shell pkg-config $(djnn-pkgconf) --libs-only-L)
install_brew: djnn_lib_path = $(subst -L, , $(djnn_lib_path))
install_brew: install


#----------------------------------------
# package builder

#deb

#note: 
# use dpkg-depcheck -d make to find out all dependency on smala
# last tryon ubuntu 18_04: 
#      	bison, flex, m4
# install with:
#		sudo dpkg -i smala_x.x.x.deb
# remove with:
#		sudo dpkg -r smala
deb_prefix_version = build/deb/smala_$(MAJOR).$(MINOR).$(MINOR2)
deb_prefix = $(deb_prefix_version)/usr
deb:	
	make -j6  install PREFIX=$(deb_prefix)
	test -d $(deb_prefix_version)/DEBIAN || mkdir -p $(deb_prefix_version)/DEBIAN
	sed -e 's,@PREFIX@,$(djnn_install_prefix),; s,@MAJOR@,$(MAJOR),; s,@MINOR@,$(MINOR),; s,@MINOR2@,$(MINOR2),' distrib/deb/control > $(deb_prefix_version)/DEBIAN/control
# cp triggers file
	cp distrib/deb/triggers $(deb_prefix_version)/DEBIAN/triggers
# remove debug symbol from library
	cd $(deb_prefix)/lib ; strip --strip-debug --strip-unneeded *.so
	cd $(deb_prefix)/bin ; strip --strip-debug --strip-unneeded *
# remove rpath from library
#	cd $(deb_prefix)/lib ; chrpath -d *.so
# build package with fakeroot
	cd "build/deb" ; fakeroot dpkg-deb --build smala_$(MAJOR).$(MINOR).$(MINOR2)
# check integrity of the build package. We still have error
#	cd "build/deb" ; lintian smala_$(MAJOR).$(MINOR).$(MINOR2).deb
.PHONY: deb

#pkg

#note: 
#      	makepkg
# install with:
#		pacman -U smala.pkg 
# remove with:
#		pacman -Rs smala
pkg_destdir_version = build/pkg/smala_$(MAJOR).$(MINOR).$(MINOR2)
ifeq ($(PREFIX),)
pkg_prefix = /usr
else
pkg_prefix = $(PREFIX)
endif
ifeq ($(DESTDIR),) 
pkg_destdir= $(shell pwd)/$(pkg_destdir_version)
else
pkg_destdir = $(DESTDIR)
endif
pkg:
#   for test only	
#	make DESTDIR=$(pkg_destdir) PREFIX=$(pkg_prefix) install
	test -d build || mkdir -p build
	sed -e 's,@MAJOR@,$(MAJOR),; s,@MINOR@,$(MINOR),;	 s,@MINOR2@,$(MINOR2),' distrib/PKGBUILD.proto > build/PKGBUILD
	cd build ; makepkg -f --skipinteg 
.PHONY: pkg

# -----------

test: $(cookbook_app_for_make_test)_test
dbg: $(cookbook_app_for_make_test)_dbg
.PHONY: test

tgz: #clean
	pushd .. ; tar czf smala/smala-src.tgz \
		smala/LICENSES smala/Makefile\
		smala/src smala/editor_modes smala/cookbook smala/doc_html ; popd

clean_not_deps:
	find build -type f -not -name "*.d" | xargs rm
.PHONY: clean_not_deps

clean_cookbook cookbook_clean:
	rm -rf $(build_dir)/cookbook
.PHONY: clean_cookbook cookbook_clean

clean_test test_clean:
	rm -rf $$(build_dir)/test
.PHONY: clean_test test_clean

distclean clear clean:
	rm -rf $(build_dir)
.PHONY: distclean clear clean


deps += $(smalac_objs:.o=.d)

ifneq ($(dep),no)
-include $(deps)
endif

pkgdeps := bison flex rtmidi nlohmann-json

ifeq ($(os),Linux)
pkgcmd := apt install -y
endif

ifeq ($(os),Darwin)
#https://brew.sh/
pkgcmd := brew install
endif

ifeq ($(os),MinGW)
#https://www.msys2.org/
# no need to prefix for bison and flex because we need msys2 version pkgdeps := $(addprefix mingw-w64-x86_64-, $(pkgdeps))
pkgcmd := pacman -Suy --needed
CXXFLAGS += -I/usr/include # Fix for FlexLexer.h in /usr/include and in /ming64/include
endif

install-pkgdeps:
	$(pkgcmd) $(pkgdeps)
.PHONY: install-pkgdeps

