#	djnn smalac compiler
#
#	The copyright holders for the contents of this file are:
#		Ecole Nationale de l'Aviation Civile, France (2017-2018)
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

all: config.mk smalac cookbook_apps
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

# remove builtin rules : speed up build process and help debug
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:

ifndef os
os := $(shell uname -s)
endif

# cross-compile support
ifndef cross_prefix
ifeq ($(os),Darwin)
cross_prefix := llvm-g
endif
#ifeq ($(os),Linux)
cross_prefix := g
#endif
#cross_prefix := arm-none-eabi-
#cross_prefix := em
#cross_prefix := i686-w64-mingw32-
endif

CC := $(cross_prefix)cc
CXX := $(cross_prefix)++
CFLAGS = -g -MMD
CXXFLAGS = $(CFLAGS) -std=c++14
LIBS = #-lboost_system

ifeq ($(os),Linux)
CXXFLAGS +=
YACC = bison -d
LD_LIBRARY_PATH=LD_LIBRARY_PATH
debugger := gdb
endif

ifeq ($(os),Darwin)
YACC = /usr/local/opt/bison/bin/bison -d
#CFLAGS += -std=c++11
LD_LIBRARY_PATH=DYLD_LIBRARY_PATH
# https://stackoverflow.com/a/33589760
debugger := /Applications/Xcode.app/Contents/Developer/usr/bin/lldb
endif

ifeq ($(os),MINGW64_NT-10.0)
YACC = bison -d
LD_LIBRARY_PATH=PATH
endif

ifeq ($(cross_prefix),em)
EMFLAGS := -Wall -Oz -s WASM=1 -s USE_SDL=2 -s FULL_ES2=1 -s ALLOW_MEMORY_GROWTH=1 -s EXPORT_ALL=1 -s ASSERTIONS=1
CFLAGS += $(EMFLAGS)

truc := \
	-I/usr/local/include \
	-I/usr/local/Cellar/expat/2.2.6/include \
	-I/usr/local/Cellar/curl/7.61.0/include \
	-I../boost_1_68_0

LDFLAGS += $(EMFLAGS) \
	-L../expat-2.2.6/lib/.libs \
	-L../curl-7.61.0/lib/.libs \
	$(djnn_lib_path_cpp)/libdjnn-animation.bc\
	$(djnn_lib_path_cpp)/libdjnn-gui.bc\
	$(djnn_lib_path_cpp)/libdjnn-display.bc\
	$(djnn_lib_path_cpp)/libdjnn-base.bc\
	$(djnn_lib_path_cpp)/libdjnn-core.bc \
	../boost_1_68_0/bin.v2/libs/chrono/build/emscripten-1.38.12/debug/cxxstd-14-iso/link-static/libboost_chrono.bc \
	../boost_1_68_0/bin.v2/libs/thread/build/emscripten-1.38.12/debug/cxxstd-14-iso/link-static/threadapi-pthread/threading-multi/libboost_thread.bc \
	../boost_1_68_0/bin.v2/libs/system/build/emscripten-1.38.12/debug/cxxstd-14-iso/link-static/libboost_system.bc \

os := em
EXE := .html
#to test: python -m SimpleHTTPServer 8080
endif


LEX = flex

# -----------
# smalac

smalac_objs := parser.o scanner.o type_manager.o j_type_manager.o c_type_manager.o cpp_type_manager.o argument.o driver.o node.o smala_native.o ctrl_node.o \
	alternative_node.o ccall_node.o operator_node.o local_node.o instruction_node.o binary_instruction_node.o native_code_node.o native_action_node.o \
	preamble.o ast.o builder.o c_builder.o j_builder.o cpp_builder.o main.o parser.o scanner.o

smalac_objs := $(addprefix $(build_dir)/src/, $(smalac_objs))

smalac := $(build_dir)/smalac

smalac: config.mk $(smalac)
.PHONY: smalac

$(smalac): $(smalac_objs)
	$(CXX) $^ -o $@

$(smalac): CFLAGS += -Isrc -I$(build_dir)/src
# -I/usr/include

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

$(build_dir)/src/scanner.o: CXXFLAGS += -Dregister=""

# for initial make -j
# find build -name "*.d" | xargs grep -s "parser.hpp" | awk '{print $1}' | awk -F "." '{print $1".o"}' | sed s/build/\$\(build_dir\)/ | xargs echo
$(build_dir)/src/scanner.o $(build_dir)/src/ast.o $(build_dir)/src/j_builder.o $(build_dir)/src/builder.o \
$(build_dir)/src/main.o $(build_dir)/src/cpp_builder.o $(build_dir)/src/c_builder.o $(build_dir)/src/parser.o $(build_dir)/src/driver.o: $(build_dir)/src/parser.hpp


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
libs_cookbook_app :=
djnn_libs_cookbook_app :=
lang_cookbook_app := cpp
include cookbook/$1/cookbook_app.mk
ckappname := $$(notdir $1)
$1_app_lang := $$(lang_cookbook_app)
$1_app_srcs_dir := cookbook/$1
$1_app_objs := $$(objs_cookbook_app)
$1_app_gensrcs := $$($1_app_objs:.o=.$$($1_app_lang))
#$1_app_srcs := $$(objs_cookbook_app)
#$1_app_objs := $$($1_app_srcs:.sma=.o)
$1_app_gensrcs := $$(addprefix $(build_dir)/cookbook/$1/, $$($1_app_gensrcs))
$1_app_objs := $$(addprefix $(build_dir)/cookbook/$1/, $$($1_app_objs))
$1_app_exe := $$(build_dir)/cookbook/$1/$$(ckappname)_app$$(EXE)
$1_app_libs := $$(addprefix -ldjnn-,$$(djnn_libs_cookbook_app)) $$(libs_cookbook_app)

ifeq ($$(lang_cookbook_app),c)
$1_app_link := $$(CC)
endif
ifeq ($$(lang_cookbook_app),cpp)
$1_app_link := $$(CXX)
endif

$$($1_app_objs): $$($1_app_gensrcs)
$$($1_app_exe): $$($1_app_objs)
	$$($1_app_link) $$^ -o $$@ $$(LDFLAGS) $$(LIBS)
$$($1_app_objs): CFLAGS += -I$$(djnn_include_path_$$($1_app_lang))
#$$(addprefix -I,$$(djnn_include_path))
$$($1_app_exe): LDFLAGS += -L$$(djnn_lib_path_$$($1_app_lang))
#$$(addprefix -L,$$(djnn_lib_path))
$$($1_app_exe): LIBS += $$($1_app_libs)

$$(notdir $1): $$($1_app_exe)
$$(notdir $1)_test: $$(notdir $1)
	(cd $$($1_app_srcs_dir); env $$(LD_LIBRARY_PATH)=$$($$(LD_LIBRARY_PATH)):$$(abspath $$(djnn_lib_path_$$($1_app_lang))) $$(shell pwd)/$$($1_app_exe))
$$(notdir $1)_dbg: $$(notdir $1)
	(cd $$($1_app_srcs_dir); env $$(LD_LIBRARY_PATH)=$$($$(LD_LIBRARY_PATH)):$$(abspath $$(djnn_lib_path_$$($1_app_lang))) $(debugger) $$(shell pwd)/$$($1_app_exe))

$$(notdir $1)_clean:
	rm $$($1_app_exe) $$($1_app_objs)


.PHONY: $1 $1_test $1_dbg

$$(notdir $1)_dbg_print:
	@echo $1_dbg
	@echo $$($1_app_objs)
	@echo $$($1_app_lang)
	@echo $$($1_app_objs:.o=.$$($1_app_lang))
	@echo $$($1_app_gensrcs)

deps += $$($1_app_objs:.o=.d)
endef

#find * -type d -mindepth 1 -maxdepth 2 | xargs echo
cookbook_apps := core/debug \
	core/paused_control \
	core/switch \
	core/text_cat \
	gui/direct_manipulation/simplest \
	gui/direct_manipulation/hysteresis \
	gui/direct_manipulation/pan_and_zoom \
	gui/direct_manipulation/pan_and_zoom_and_drag \
	gui/direct_manipulation/sketching_simple \
	gui/direct_manipulation/sketching_advanced \
	gui/fitts_law \
	gui/widgets/checkbox \
	gui/widgets/tab \
	gui/widgets/dial \
	gui/widgets/button \
	gui/animation/path_animation \
	network/helloIvy

cookbook_apps: $(notdir $(cookbook_apps))
.PHONY: cookbook_apps $(notdir $(cookbook_apps))

$(foreach a,$(cookbook_apps),$(eval $(call cookbookapp_makerule,$a)))

# .sma to .cpp
$(build_dir)/%.cpp $(build_dir)/%.h: %.sma $(smalac)
	@mkdir -p $(dir $@)
	@echo smalac $<
	$(smalac) -g $< || (c=$$?; rm -f $*.cpp $*.h $*.java; (exit $$c))
	@mv $*.cpp $(build_dir)/$(*D)
	@if [ -f $*.h ]; then mv $*.h $(build_dir)/$(*D); fi;

# .sma to .c
$(build_dir)/%.c $(build_dir)/%.h: %.sma $(smalac)
	@mkdir -p $(dir $@)
	@echo smalac $<
	$(smalac) -c $< || (c=$$?; rm -f $*.c $*.h $*.java; (exit $$c))
	@mv $*.c $(build_dir)/$(*D)
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

clean_tests:
	rm -rf $(build_dir)/cookbook
.PHONY: clean_test


distclean clear clean:
	rm -rf $(build_dir)
.PHONY: distclean clear clean

truc:
	@echo $(deps)

deps += $(smalac_objs:.o=.d)
-include $(deps)


pkgdeps := bison flex

ifeq ($(os),Darwin)
#https://brew.sh/
pkgcmd := brew install
endif

ifeq ($(os),MINGW64_NT-10.0)
#https://www.msys2.org/
# no need prefix for bison and flex because we need msys2 version pkgdeps := $(addprefix mingw-w64-x86_64-, $(pkgdeps))
pkgcmd := pacman -S
endif

install-pkgdeps:
	$(pkgcmd) $(pkgdeps)
.PHONY: install-pkgdeps

