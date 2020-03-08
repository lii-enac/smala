# canonical Makefile for smala applications
# 1. copy stand_alone directory somwhere: cp -r cookbook/stand_alone /some/where/else
# 2. edit configuration part (executable name, srcs, target_lang, djnn_libs, path to djnn-cpp and smalac)
# 3. make test

redirect: default
.PHONY: redirect

#--
# configuration

# cpp example
exe := button
srcs_sma := src/Button.sma src/main.sma
#srcs_sma := $(shell find $(src_dir) -name "*.sma")
srcs_other := src/foo.cpp
#srcs_other := $(shell find $(src_dir) -name "*.cpp")

#djnn_libs := gui display animation utils comms base core
djnn_libs := gui display base core

#relative path
src_dir := src
build_dir := build
res_dir := res
exe_dir := .

# in cookbook
djnn_path := ../../../djnn-cpp
# standalone
#djnn_path := ../djnn-cpp
djnn_include_path_cpp := $(djnn_path)/src
djnn_lib_path_cpp := $(djnn_path)/build

# in cookbook
smala_path := ../..
# standalone
#smala_path := ../smala
smalac := $(smala_path)/build/smalac

# for emscripten
em_ext_libs_path := ../../../djnn-emscripten-ext-libs


#djnn_path := ../../../local-install
#djnn_java_classpath := ../../../djnn-java/src:../../../djnn-java/src/jna.jar
# java example
#exe := MyApp
#java_package := truc
#srcs := Button.sma MyApp.sma
#target_lang := java
target_lang := cpp


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

CXXFLAGS += -MMD -g -std=c++14

ifeq ($(os),Linux)
LD_LIBRARY_PATH=LD_LIBRARY_PATH
debugger := gdb
endif

ifeq ($(os),Darwin)
LD_LIBRARY_PATH=DYLD_LIBRARY_PATH
# https://stackoverflow.com/a/33589760
debugger := PATH=/usr/bin /Applications/Xcode.app/Contents/Developer/usr/bin/lldb
endif

ifeq ($(os),MinGW)
LD_LIBRARY_PATH=PATH
debugger := gdb
endif

ifeq ($(os),em)
LD_LIBRARY_PATH=LD_LIBRARY_PATH
debugger := gdb
EXE := .html
endif

exe := $(exe)$(EXE)

# -- cpp

ifeq ($(target_lang),cpp)

exe := $(build_dir)/$(exe)

default: $(exe)
.PHONY: default

test: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)="$(abspath $(djnn_lib_path_cpp))":$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)")
dbg: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)="$(abspath $(djnn_lib_path_cpp))":$$$(LD_LIBRARY_PATH) $(debugger) "$(shell pwd)/$(exe)")
.PHONY: test

LD  = $(CXX)

objs_sma := $(srcs_sma:.sma=.o)
objs_sma := $(addprefix $(build_dir)/,$(objs_sma))
objs_other := $(srcs_other:.$(target_lang)=.o)
objs_other := $(addprefix $(build_dir)/,$(objs_other))

objs := $(objs_sma) $(objs_other)

gensrcs := $(objs_sma:.o=.$(target_lang))
#$(objs_sma): $(gensrcs) # this forces the right language to compile the generated sources, but it will rebuild all sma files


ifeq ($(cross_prefix),em)
app_libs := $(addsuffix .bc,$(addprefix $(djnn_lib_path_cpp)/libdjnn-,$(djnn_libs)))
else
app_libs := $(addprefix -ldjnn-,$(djnn_libs))
endif


$(objs): CXXFLAGS += -I$(djnn_include_path_cpp) -I$(src_dir) -I$(build_dir)/$(src_dir)
$(exe): LDFLAGS += -L$(djnn_lib_path_cpp)
$(exe): LIBS += $(app_libs)

$(exe): $(objs)
	@mkdir -p $(dir $@)
	$(LD) $^ -o $@ $(LDFLAGS) $(LIBS)

# .sma to .cpp, .c etc
$(build_dir)/%.$(target_lang) $(build_dir)/%.h: %.sma
	@mkdir -p $(dir $@)
	@echo smalac $<
	@$(smalac) -$(target_lang) $<
	@mv $*.$(target_lang) $(build_dir)/$(*D)
	@if [ -f $*.h ]; then mv $*.h $(build_dir)/$(*D); fi;

#@if [ -f $*.h ] && ! cmp -s $*.h $(build_dir)/$*.h; then mv $*.h $(build_dir)/$(*D); fi;

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

endif


# -- java

ifeq ($(target_lang),java)

.PHONY: $(exe)

build_dir_old := $(build_dir)
build_dir := $(build_dir)/$(java_package)
exe_full := $(build_dir)/$(exe)

default: $(exe_full)
.PHONY: default

test: $(exe_full)
	java -classpath $(djnn_java_classpath):$(build_dir_old) -Djna.library.path=$(djnn_path)/lib -XstartOnFirstThread $(java_package)/$(exe)
.PHONY: test

classes := $(srcs:.sma=.class)
classes := $(addprefix $(build_dir)/,$(classes))
srcs_java := $(srcs:.sma=.java)
srcs_java := $(addprefix $(build_dir)/,$(srcs_java))


$(exe_full): $(classes)
$(classes): $(srcs_java)
	javac -classpath $(djnn_java_classpath) $?

# .sma
$(build_dir)/%.java: %.sma
	@mkdir -p $(dir $@)
	@echo $(smalac) -j -package $(java_package) $<
	@$(smalac) -j -package $(java_package) $< || (c=$$?; rm -f $*.c $*.h $*.java; (exit $$c))
	@if [ -f $*.java ]; then mv $*.java $(build_dir)/$(*D); fi;

endif

# --

distclean clear:
	rm -rf build
clean:
	rm -f $(gensrcs) $(objs) $(deps) $(classes) $(srcs_java)
.PHONY: clean clear distclean

foo:
	echo $(objs_other)

