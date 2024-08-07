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
project_dir := project/make

include $(project_dir)/0-start.mk

# determine host os
include $(project_dir)/1-os.mk

include $(project_dir)/2-config.mk

# include user-specified config if present
-include config.mk

# default config
#include $(project_dir)/config.default.mk

# utils
include $(project_dir)/4-utils.mk

# # sources
# include srcs.mk

# # main stuff
# include $(project_dir)/5-main.mk

include $(project_dir)/10-end.mk


# ---------------------------------------
# save user-provided CXXFLAGS, and use CXXFLAGS as the ultimate compiler configuration

CXXFLAGS_CFG := $(CXXFLAGS)
CXXFLAGS :=


CXXLD ?= $(CXX)

# CXX_SC is the compiler for the smalac compiler
CXX_SC := $(CXX)

# CC_CK and CXX_CK are the compilers for cookbooks
ifndef CC_CK
CC_CK := $(CC)
endif
ifndef CXX_CK
CXX_CK := $(CXX)
endif

lib_smala_name = libsmala

ifneq ($(djnn-pkgconf),)
pkgexists := $(shell pkg-config $(djnn-pkgconf) --exists; echo $$?)
ifeq ($(pkgexists),0)
djnn_cflags = $(shell pkg-config $(djnn-pkgconf) --cflags)
djnn_ldflags = $(shell pkg-config $(djnn-pkgconf) --libs-only-L)
djnn_ldlibs = $(shell pkg-config $(djnn-pkgconf) --libs-only-l)
djnn_libs = $(shell pkg-config $(djnn-pkgconf) --libs)
djnn_lib_path = $(shell pkg-config $(djnn-pkgconf) --libs-only-L)
djnn_lib_path := $(subst -L, , $(djnn_lib_path))
djnn_include_path_only = $(subst -I, , $(djnn_cflags))
endif
endif

ifneq ($(pkgexists),0)
ifneq ($(djnn_path),)
djnn_cflags = -I$(djnn_path)/src
djnn_ldflags = -L$(djnn_path)/build/lib
#djnn_ldlibs := -ldjnn-core -ldjnn-base -ldjnn-animation -ldjnn-audio -ldjnn-comms -ldjnn-display -ldjnn-exec_env -ldjnn-files -ldjnn-gui -ldjnn-input -ldjnn-utils
djnn_modules ?= animation comms gui display input files utils base exec_env core
djnn_ldlibs = $(addprefix -ldjnn-,$(djnn_modules))
djnn_libs = $(djnn_ldflags) $(djnn_ldlibs)
djnn_lib_path = $(djnn_path)/build/lib
djnn_include_path_only = $(djnn_path)/src
else
error no djnn_cpp path declared
endif
endif

#undefine pkgexists

djnn_libs_SL := $(djnn_libs)

# ----------------------------------

CFLAGS_COMMON += -MMD

CXXFLAGS_COMMON += $(CFLAGS_COMMON) -std=c++17

CXXFLAGS_SC += -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-vla-extension
CXXFLAGS_SC += $(CXXFLAGS_COMMON) -Isrc -I$(build_dir)/src -I$(build_dir)/lib
# for filesystem.h
CXXFLAGS_SC += $(djnn_cflags)
LDFLAGS_SC += $(LDFLAGS_COMMON)

CXXFLAGS_CK += $(CXXFLAGS_COMMON) $(djnn_cflags)
LDFLAGS_CK += $(LDFLAGS_COMMON)

# -----------------------------------
ifeq ($(os),Linux)
compiler ?= gnu
linker ?= gnu
CFLAGS_COMMON +=  -fpic
YACC ?= bison -d -W
LD_LIBRARY_PATH=LD_LIBRARY_PATH
debugger := gdb
lib_suffix =.so
#DYNLIB ?= -shared
LDFLAGS_SC += -lstdc++fs
endif

ifeq ($(os),Darwin)
compiler ?= llvm
linker ?= llvm
ifeq ($(PREFIX),)
brew_prefix := $(shell brew --prefix)
else
brew_prefix := $(HOMEBREW_PREFIX)
endif
ifeq ($(origin YACC), default)
YACC := $(brew_prefix)/opt/bison/bin/bison -d -Wno-conflicts-sr
endif
ifeq ($(origin LEX), default)
LEX := $(brew_prefix)/opt/flex/bin/flex
endif
LD_LIBRARY_PATH=DYLD_LIBRARY_PATH
# https://stackoverflow.com/a/33589760
debugger := PATH=/usr/bin /Applications/Xcode.app/Contents/Developer/usr/bin/lldb
#other_runtime_lib_path := /Users/conversy/src-ext/SwiftShader/build
#other_runtime_lib_path += /Users/conversy/recherche/istar/code/misc/MGL/build
#other_runtime_lib_path += /Users/conversy/recherche/istar/code/misc/mesa/builddir/src/gallium/targets/osmesa
#other_runtime_lib_path += /Users/conversy/recherche/istar/code/misc/SDL2-2.28.1/local/lib
#CFLAGS_COMMON += -isysroot $(shell xcode-select -p)/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk
CXXFLAGS_SC += -I$(shell brew --prefix flex)/include
LDFLAGS_SC += -L$(shell brew --prefix flex)/lib
#lib_suffix =.dylib
#DYNLIB ?= -dynamiclib
endif

ifeq ($(os),MinGW)
compiler ?= gnu
linker ?= gnu
CFLAGS_COMMON += -fpic
YACC ?= bison -d -Wno-conflicts-sr
LD_LIBRARY_PATH=PATH
debugger := gdb
lib_suffix =.dll
#DYNLIB ?= -shared
endif

LD_LIBRARY_PATH ?= LD_LIBRARY_PATH

ifeq ($(linker),gnu)
#lib_suffix =.so
DYNLIB ?= -shared
endif
ifeq ($(linker),llvm)
#lib_suffix =.dylib
DYNLIB ?= -dynamiclib
endif
ifeq ($(linker),mold)
DYNLIB ?= -shared
endif


ifeq ($(cookbook_cross_prefix),arm-none-eabi-)
CXXFLAGS_CK += -Wno-psabi #https://stackoverflow.com/a/48149400
endif


YACC ?= bison
LEX ?= flex

CXXLD_CK ?= $(CXX_CK)

# -----------
# smalac

bin_name := smalac

smalac_objs := parser.o scanner.o type_manager.o cpp_type_manager.o argument.o driver.o node.o smala_native.o ctrl_node.o \
	newvar_node.o instruction_node.o binary_instruction_node.o expr_node.o name_context.o \
	native_expression_node.o native_component_node.o range_node.o set_parent_node.o transition_node.o preamble.o ast.o \
	js_type_manager.o js_builder.o html_builder.o builder.o cpp_builder.o main.o scanner.o \
	process_class_path.o

$(build_dir)/src/process_class_path.cpp:
ifeq ($V,max)
	@mkdir -p $(dir $@)
	sh tools/make_process_class_path $@ $(djnn_include_path_only)
else
	@$(call rule_message,generating,$(stylized_target))
	@mkdir -p $(dir $@)
	@sh tools/make_process_class_path $@ $(djnn_include_path_only)
endif


smalac_objs := $(addprefix $(build_dir)/src/, $(smalac_objs))

smalac := $(build_dir)/$(bin_name)

smalac: $(smalac)
.PHONY: smalac

$(smalac): $(smalac_objs)
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(CXXLD) $^ -o $@ $(LDFLAGS)
else
	@$(call rule_message,linking to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(CXXLD) $^ -o $@ $(LDFLAGS)
endif
#$(CXXLD) $^ -o $@ $(LDFLAGS)

$(smalac): CXX = $(CXX_SC)
$(smalac): LDFLAGS += $(LDFLAGS_SC)
#'override" necessary to make 'make -j lib' work in case smalac should be rebuilt
$(smalac_objs): override CXXFLAGS = $(CXXFLAGS_CFG) $(CXXFLAGS_SC)

# ------------
# smala lib

smala_src_lib_dir := src_lib
smala_dst_lib_dir := $(build_dir)/lib
smala_lib_header_dir := $(build_dir)/$(smala_src_lib_dir)
smala_lib := $(smala_dst_lib_dir)/$(lib_smala_name)$(lib_suffix)
smala_lib_srcs := $(shell find $(smala_src_lib_dir) -name "*.sma")
smala_lib_objs := $(addprefix $(build_dir)/, $(patsubst %.sma,%.o,$(smala_lib_srcs)))
smala_lib_headers := $(addprefix $(build_dir)/, $(patsubst %.sma,%.h,$(smala_lib_srcs)))

smala_lib_path ?= $(smala_dst_lib_dir)

$(smala_lib_objs): CXX = $(CXX_CK)
$(smala_lib_objs): CXXFLAGS = $(CXXFLAGS_CFG) $(CXXFLAGS_PCH_DEF) $(CXXFLAGS_PCH_INC) $(CXXFLAGS_CK) -Ibuild/src_lib

# find build -name "*.d" | xargs grep -s "gui/widgets/IWidget.h" | awk '{print $1}' | awk -F "." '{print $1".o"}' | sed s/build/\$\(build_dir\)/ | xargs echo
# find build/src_lib -name "*.d" | xargs grep -h build/src_lib | xargs echo
# find build/src_lib -name "*.d" | xargs grep -h build/src_lib | grep -v cpp
build/src_lib/gui/widgets/StandAloneComboBox.o: build/src_lib/gui/animation/Animator.h build/src_lib/gui/widgets/IWidget.h build/src_lib/gui/widgets/ComboBoxItem.h
build/src_lib/gui/widgets/RadioButton.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/Label.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/VSlider.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/HSpace.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/PushButton.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/VBox.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/ToggleButton.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/ComboBox.o: build/src_lib/gui/widgets/IWidget.h build/src_lib/gui/widgets/ComboBoxItem.h
build/src_lib/gui/widgets/VSpace.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/HBox.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/CheckBox.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/HSlider.o: build/src_lib/gui/widgets/IWidget.h
build/src_lib/gui/widgets/UITextField.o: build/src_lib/gui/widgets/IWidget.h

$(smala_lib): $(smala_lib_objs)
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(CXXLD_CK) $(DYNLIB) -o $(notdir $@) $^ $(LDFLAGS_CK) $(djnn_libs_SL)
	mv $(notdir $@) $@
else
	@$(call rule_message,linking to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(CXXLD_CK) $(DYNLIB) -o $(notdir $@) $^ $(LDFLAGS_CK) $(djnn_libs_SL)
	@mv $(notdir $@) $@
endif

#@mkdir -p $(dir $@)
#$(CXXLD_CK) $(DYNLIB) -o $(notdir $@) $^ $(LDFLAGS_CK) $(djnn_libs_SL)
	

smala_lib: $(smala_lib)
.PRECIOUS: $(smala_lib_headers)

lib: smala_lib

ifeq ($(keep_intermediate),yes)
.SECONDARY:
endif


# ------------
# automatic rules


$(build_dir)/src/parser.cpp $(build_dir)/src/parser.hpp $(build_dir)/src/location.hh: src/parser.y
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(YACC) -o $(build_dir)/src/parser.cpp $<
else
	@$(call rule_message,compiling to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(YACC) -o $(build_dir)/src/parser.cpp $<
endif

$(build_dir)/%.cpp: %.l
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(LEX) -o $@ $<
else
	@$(call rule_message,compiling to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(LEX) -o $@ $<
endif

	

# ------------
# specific dependencies and peculiarities

$(build_dir)/src/scanner.o: CXXFLAGS += -Dregister=""
#$(build_dir)/src/location.hh: src/parser.y #$(build_dir)/src/parser.cpp
$(build_dir)/src/parser.o: CXXFLAGS_COMMON += -Wno-unused-but-set-variable

# for initial make -j
# find build -name "*.d" | xargs grep -s "parser.hpp" | awk '{print $1}' | awk -F "." '{print $1".o"}' | sed s/build/\$\(build_dir\)/ | xargs echo
$(build_dir)/src/scanner.o $(build_dir)/src/ast.o $(build_dir)/src/builder.o $(build_dir)/src/cpp_builder.o \
$(build_dir)/src/main.o $(build_dir)/src/parser.o $(build_dir)/src/js_builder.o \
$(build_dir)/src/driver.o: $(build_dir)/src/parser.hpp

# find build -name "*.d" | xargs grep -s "location.hh" | awk '{print $1}' | awk -F "." '{print $1".o"}' | sed s/build/\$\(build_dir\)/ | xargs echo
$(build_dir)/src/scanner.o $(build_dir)/src/ast.o $(build_dir)/src/newvar_node.o $(build_dir)/src/range_node.o \
$(build_dir)/src/native_expression_node.o $(build_dir)/src/native_component_node.o $(build_dir)/src/builder.o \
$(build_dir)/src/smala_native.o $(build_dir)/src/cpp_builder.o $(build_dir)/src/preamble.o $(build_dir)/src/instruction_node.o \
$(build_dir)/src/set_parent_node.o $(build_dir)/src/expr_node.o $(build_dir)/src/name_context.o $(build_dir)/src/main.o \
$(build_dir)/src/binary_instruction_node.o $(build_dir)/src/js_builder.o $(build_dir)/src/driver.o \
$(build_dir)/src/node.o $(build_dir)/src/ctrl_node.o $(build_dir)/src/transition_node.o \
$(build_dir)/src/parser.o: $(build_dir)/src/location.hh


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

$(merr_objs):
#CXXFLAGS += -std=c++1z 

$(merr): $(merr_objs)
	$(CXX) $^ -o $@

$(build_dir)/src/parser.cpp: src/errors.h


#include project/make/7-pch.mk
# ---------------------------------------
# precompiled headers

# https://stackoverflow.com/questions/58841/precompiled-headers-with-gcc
# https://stackoverflow.com/questions/26755219/how-to-use-pch-with-clang

compiler ?= llvm

ifneq ($(use_pch),no)

ifeq ($(compiler),llvm)
pch_ext = .pch
endif
ifeq ($(compiler),gnu)
pch_ext = .gch
endif

pch_file := precompiled.h
pch_src := src_lib/$(pch_file)
pch_dst := $(build_dir)/src_lib/$(pch_file)$(pch_ext)

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
	@$(CXX) -x c++-header $(CXXFLAGS) $< -o $@
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

$(pch_dst): override CXXFLAGS = $(CXXFLAGS_CFG) $(CXXFLAGS_CK) $(CXXFLAGS_PCH_DEF)

pch: $(pch_dst)
clean_pch:
	rm -f $(pch_dst)

endif

# -----------
# cookbook apps

ld_library_path := $(call join-with,:,$(ld_library_path))
ld_library_path := $(ld_library_path):$(abspath $(djnn_lib_path)):$(abspath $(smala_lib_path))

define cookbookapp_makerule
djnn_libs_cookbook_app :=
smala_libs_cookbook_app :=
pkg_cookbook_app :=
libs_cookbook_app :=
cflags_cookbook_app :=
cppflags_cookbook_app :=
res_dir :=
other_runtime_lib_path ?=

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

#ifeq ($$(cookbook_cross_prefix),em)
ifeq ($$(os),em)
$1_app_libs := $$(addsuffix .bc,$$(addprefix $$(djnn_lib_path)/libdjnn-,$$(djnn_libs_cookbook_app))) $$(libs_cookbook_app)

#	--preload-file $$($1_app_srcs_dir)/$$($1_res_dir)@$$($1_res_dir)
#	--preload-file /Library/Fonts/Arial.ttf@/usr/share/fonts/Arial.ttf
else
$1_app_libs := $$(addprefix -ldjnn-,$$(djnn_libs_cookbook_app)) $$(libs_cookbook_app)
endif

ifneq ($$(smala_libs_cookbook_app),)
$1_app_cppflags += -I$(smala_lib_header_dir)
#ifeq ($$(cookbook_cross_prefix),em)
ifeq ($$(os),em)
$1_app_libs += $$(build_dir)/lib/lib$$(smala_libs_cookbook_app)$(lib_suffix)
else
$1_app_libs += -Lbuild/lib $$(addprefix -l,$$(smala_libs_cookbook_app))
ifeq ($(os),Linux)
$1_app_libs += $$(call uniq,$$(djnn_libs) $$($1_app_libs)) #$(djnn_libs) is necessary for linux ld
endif
endif
$$($1_app_objs): $$(smala_lib)

$$(notdir $1)_toto:
	echo $$(smala_lib)
endif

ifneq ($$($1_app_pkg),)
#$1_lib_pkgpath = $$(subst $$() $$(),:,$$(lib_pkgpath))
$1_app_cppflags += $$(shell env PKG_CONFIG_PATH=$$(PKG_CONFIG_PATH):$$($1_lib_pkgpath) pkg-config --cflags $$($1_app_pkg))
$1_app_libs += $$(shell env PKG_CONFIG_PATH=$$(PKG_CONFIG_PATH):$$($1_lib_pkgpath) pkg-config --libs $$($1_app_pkg))
endif

$1_app_link := $$(CXXLD_CK)

#$$($1_app_objs): $$($1_app_gensrcs)
$$($1_app_objs): CC = $$(CC_CK)
$$($1_app_objs): CXX = $$(CXX_CK)
$$($1_app_objs): CXXFLAGS_CK += $$($1_app_cppflags) $$($1_app_cflags)
$$($1_app_exe): LDFLAGS_CK += $$(djnn_ldflags)
$$($1_app_exe): LIBS += $$($1_app_libs)

$$($1_app_exe): $$($1_app_objs)
ifeq ($V,max)
	@mkdir -p $$(dir $$@)
	$$($1_app_link) $$^ -o $$@ $$(LDFLAGS_CK) $$(LIBS)
else
	@$$(call rule_message,linking to,$$(stylized_target))
	@mkdir -p $$(dir $$@)
	@$$($1_app_link) $$^ -o $$@ $$(LDFLAGS_CK) $$(LIBS)
endif
#@#$$($1_app_link) $$^ -o $$@ $$(LDFLAGS_CK) `env PKG_CONFIG_PATH=$$(djnn_lib_path)/.. pkg-config --libs --static djnn-cpp`

$$(notdir $1)_objs: $$($1_app_objs)

$$(notdir $1): $$($1_app_exe)

$$(notdir $1)_test: $$(notdir $1)
	(cd "$$($1_app_srcs_dir)"; env $$(LD_LIBRARY_PATH)="$$($$(LD_LIBRARY_PATH)):$$(ld_library_path):$$(call join-with,:,$$($1_other_runtime_lib_path))" $$(launch_cmd) "$$(shell pwd)/$$($1_app_exe)")
$$(notdir $1)_dbg: $$(notdir $1)
	(cd "$$($1_app_srcs_dir)"; env $$(LD_LIBRARY_PATH)="$$($$(LD_LIBRARY_PATH)):$$(ld_library_path):$$(call join-with,:,$$($1_other_runtime_lib_path))" $$(debugger) "$$(shell pwd)/$$($1_app_exe)")

$$(notdir $1)_clean:
	rm -f $$($1_app_exe) $$($1_app_objs) $$($1_app_gensrcs)
$$(notdir $1)_clear:
	rm -rf $(build_dir)/cookbook/$1

.PHONY: $1 $1_test $1_dbg

$$(notdir $1)_dbg_print:
	@echo $1_dbg
	@echo $$($1_app_objs)
	@echo $$($1_app_gensrcs)

app_deps += $$($1_app_objs:.o=.d)
app_objs += $$($1_app_objs)
endef


cookbook_apps := $(shell cd cookbook && find * -name cookbook_app.mk | xargs -I{} dirname {})
disable_cookbook_apps ?= \
	extra/crazyflie \
	extra/crazyflie_drone_app \
	comms/swim \
	comms/midi \
	gui/physics \
	gui/graphics/video \
	comms/ros2 \
	core/bidi \
	gui/direct_manipulation/rotate_resize

cookbook_apps := $(filter-out $(disable_cookbook_apps),$(cookbook_apps))

cookbook_apps += $(cookbook_apps_extra)

$(foreach a,$(cookbook_apps),$(eval $(call cookbookapp_makerule,$a)))

cookbook_apps: $(notdir $(cookbook_apps))
cookbook_apps_test: $(addsuffix _test,$(notdir $(cookbook_apps)))


# precompiled header deps
$(app_objs): $(pch_dst)
$(smala_lib_objs): $(pch_dst)

$(app_objs): CXXFLAGS = $(CXXFLAGS_CFG) $(CXXFLAGS_CK) $(CXXFLAGS_PCH_DEF) $(CXXFLAGS_PCH_INC)


# ---------------------------------------
# generate ninja file from Makefile

# make2ninja ?= tools/make2ninja.py

# ninja: clear build.ninja
# build.ninja: $(make2ninja)
# 	$(MAKE) -Bnd V=max | python3 $(make2ninja) > build.ninja
# .PHONY: build.ninja ninja

# $(make2ninja):
# 	mkdir -p $(dir $@)
# 	curl -O https://raw.githubusercontent.com/conversy/make2ninja/master/make2ninja.py && mv make2ninja.py $(make2ninja)
# ifeq ($(os),Darwin)
# 	sed -i '' "s/\'(./\`(./" $@
# endif

# ---------------------------------------
# stand alone

stand_alone_dir ?= ../stand_alone
stand_alone:
	mkdir $(stand_alone_dir)
	cp -r cookbook/stand_alone/. $(stand_alone_dir)



include project/make/8-rules.mk

# ---------------------------------------
# rules

# .sma to .js
$(build_dir)/%.js: %.sma $(smalac)
	@mkdir -p $(dir $@)
	$(smalac) -lang=js $<
	@mv $*.js $(build_dir)/$(*D)

$(build_dir)/%.html: %.sma $(smalac)
	@mkdir -p $(dir $@)
	$(smalac) -lang=js $<
	@mv $*.js $(build_dir)/$(*D)

# # .sma to .cpp
# $(build_dir)/%.cpp $(build_dir)/%.h: %.sma $(smalac)
# 	@mkdir -p $(dir $@)
# 	$(smalac) $(SMAFLAGS) -cpp $< -builddir $(build_dir)

# # from .c user sources
# $(build_dir)/%.o: %.c
# 	@mkdir -p $(dir $@)
# 	$(CC) $(CFLAGS) -c $< -o $@

# # from .cpp user sources
# $(build_dir)/%.o: %.cpp
# 	@mkdir -p $(dir $@)
# 	$(CXX) $(CXXFLAGS) -c $< -o $@

# # for .c generated sources
# $(build_dir)/%.o: $(build_dir)/%.c
# 	@mkdir -p $(dir $@)
# 	$(CC) $(CFLAGS) -c $< -o $@

# # for .cpp generated sources
# $(build_dir)/%.o: $(build_dir)/%.cpp
# 	@mkdir -p $(dir $@)
# 	$(CXX) $(CXXFLAGS) -c $< -o $@


# ---------------------------------------
# package config

ifeq ($(PREFIX),)
# dev install
smala_install_prefix :=  $(abspath $(build_dir))
ifeq ($(os),Darwin)
pkg_config_install_prefix := /usr/local
else
pkg_config_install_prefix := /usr
endif
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
	@sed -e 's,@PREFIX@,$(pkg_config_install_prefix),; s,@MAJOR@,$(MAJOR),; s,@MINOR@,$(MINOR),; s,@MINOR2@,$(MINOR2),' $< > $@


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


ifeq ($(os),Linux)
vscode_home = $(HOME)/.vscode/extensions
sublime_home = $(HOME)/.config/sublime-text-3/Packages/User
endif

ifeq ($(os),Darwin)
vscode_home = $(HOME)/.vscode/extensions
sublime_home = "$(HOME)/Library/Application Support/Sublime Text/Packages/User"
endif

ifeq ($(os),MinGW)
vscode_home = $(shell cygpath -u "$(USERPROFILE)/.vscode/extensions")
sublime_home = $(shell cygpath -u "$(USERPROFILE)/AppData/Roaming/Sublime Text 3/Packages/User")
endif

install_editor_modes:
	if [ -d $(vscode_home) ]; then \
		cp -r editor_modes/vscode/smala $(vscode_home); \
	fi
	if [ -d $(sublime_home) ]; then \
		cp editor_modes/sublime/smala.sublime-syntax editor_modes/sublime/smala.tmPreferences editor_modes/sublime/Make-color.sublime-build $(sublime_home); \
	fi

install: default smala_lib install_pkgconf install_headers install_libs install_bin install_editor_modes



# we have to redefine all variables already computed in config.mk
install_brew: install
install_brew: djnn_cflags = $(shell pkg-config $(djnn-pkgconf) --cflags)
install_brew: djnn_ldflags = $(shell pkg-config $(djnn-pkgconf) --libs-only-L)
install_brew: djnn_ldlibs = $(shell pkg-config $(djnn-pkgconf) --libs-only-l)
install_brew: djnn_libs = $(shell pkg-config $(djnn-pkgconf) --libs)
install_brew: djnn_lib_path = $(shell pkg-config $(djnn-pkgconf) --libs-only-L)
install_brew: djnn_lib_path = $(subst -L, , $(djnn_lib_path))
install_brew: djnn_include_path_only = $(subst -I, , $(djnn_cflags))


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

MINOR3 = $(shell echo `date +%j`)
deb_git_prefix_version = build/deb/smala_$(MAJOR).$(MINOR).$(MINOR2).$(MINOR3)
deb_git_prefix = $(deb_git_prefix_version)/usr
deb_git:
	make -j6  install PREFIX=$(deb_git_prefix)
	test -d $(deb_git_prefix_version)/DEBIAN || mkdir -p $(deb_git_prefix_version)/DEBIAN
	sed -e 's,@PREFIX@,$(djnn_install_prefix),; s,@MAJOR@,$(MAJOR),; s,@MINOR@,$(MINOR),; s,@MINOR2@,$(MINOR2).$(MINOR3),' distrib/deb/control > $(deb_git_prefix_version)/DEBIAN/control
	cp distrib/deb/triggers $(deb_git_prefix_version)/DEBIAN/triggers
	cd $(deb_git_prefix)/lib ; strip --strip-debug --strip-unneeded *.so
	cd $(deb_git_prefix)/bin ; strip --strip-debug --strip-unneeded *
	cd "build/deb" ; fakeroot dpkg-deb --build smala_$(MAJOR).$(MINOR).$(MINOR2).$(MINOR3)

.PHONY: deb deb_git


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

clean_cookbook cookbook_clear:
	rm -rf $(build_dir)/cookbook
clean_lib lib_clean:
	@#rm -rf $(build_dir)/$(src_lib)
	rm -f $(smala_lib) $(smala_lib_objs)

.PHONY: clean_cookbook cookbook_clear

clean_test test_clean:
	rm -rf $$(build_dir)/test
.PHONY: clean_test test_clean

# distclean clear clean:
# 	rm -rf $(build_dir)
# .PHONY: distclean clear clean


deps += $(smalac_objs:.o=.d)

ifneq ($(dep),no)
-include $(deps)
endif

pkgdeps := bison flex

ifeq ($(os),Linux)
pkgcmd := apt install -y
pkgdeps += librtmidi-dev nlohmann-json3-dev
endif

ifeq ($(os),Darwin)
#https://brew.sh/
pkgcmd := brew install
pkgdeps += rtmidi nlohmann-json
endif

ifeq ($(os),MinGW)
#https://www.msys2.org/
# no need to prefix for bison and flex because we need msys2 version pkgdeps := $(addprefix mingw-w64-x86_64-, $(pkgdeps))
pkgcmd := pacman -Suy --needed
CXXFLAGS_SC += -I/usr/include # Fix for FlexLexer.h in /usr/include and in /ming64/include
pkgdeps += mingw-w64-x86_64-nlohmann-json mingw-w64-x86_64-rtmidi
endif

install-pkgdeps:
	$(pkgcmd) $(pkgdeps)
.PHONY: install-pkgdeps

