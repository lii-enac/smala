# canonical Makefile for smala applications
# 1. copy stand_alone directory somwhere: cp -r cookbook/stand_alone /some/where/else
# 2. edit configuration part (executable name, srcs, target_lang, djnn_libs, path to djnn-cpp and smalac)
# 3. make test

#--
# configuration

# cpp example
exe := button
srcs := src/Button.sma src/main.sma
target_lang := cpp

# java example
#exe := MyApp
#java_package := truc
#srcs := Button.sma MyApp.sma
#target_lang := java

djnn_libs := gui base core

src_dir := src
build_dir := build

smala_path := ../..
smalac := $(smala_path)/build/smalac

djnn_path := ../../../djnn-cpp
djnn_include_path_cpp := $(djnn_path)/src
djnn_lib_path_cpp := $(djnn_path)/build

#djnn_path := ../../../local-install
#djnn_java_classpath := ../../../djnn-java/src:../../../djnn-java/src/jna.jar


# -------------------------------------------------------------------
# hopefully no need to tweak the lines below

# remove builtin rules: speed up build process and help debug
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:

ifndef os
os := $(shell uname -s)
endif

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

# -- c

ifeq ($(target_lang),cpp)

exe := $(build_dir)/$(exe)

default: $(exe)
.PHONY: default

test: $(exe)
	(cd $(src_dir); env $(LD_LIBRARY_PATH)=$(abspath $(djnn_lib_path_cpp)) $(shell pwd)/$(exe))
.PHONY: test

CXXFLAGS ?= -g -std=c++14
LD  = $(CXX)

objs := $(srcs:.sma=.o)
objs := $(addprefix $(build_dir)/,$(objs))
gensrcs := $(objs:.o=.$(target_lang))
$(objs): $(gensrcs) # this forces the right language to compile the generated sources

$(objs): CXXFLAGS += -I$(djnn_include_path_cpp) -MMD
$(exe): LDFLAGS += -L$(djnn_lib_path_cpp)
$(exe): LIBS += $(addprefix -ldjnn-,$(djnn_libs))

$(exe): $(objs)
	@mkdir -p $(dir $@)
	$(LD) $^ -o $@ $(LDFLAGS) $(LIBS)

# .sma to .cpp
$(build_dir)/%.cpp $(build_dir)/%.h: %.sma $(smalac)
	@mkdir -p $(dir $@)
	@echo smalac $<
	$(smalac) -cpp $< || (c=$$?; rm -f $*.cpp $*.h $*.java; (exit $$c))
	@mv $*.cpp $(build_dir)/$(*D)
	@if [ -f $*.h ]; then mv $*.h $(build_dir)/$(*D); fi;

# .sma to .c
$(build_dir)/%.c $(build_dir)/%.h: %.sma $(smalac)
	@mkdir -p $(dir $@)
	@echo moncul smalac $<
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


deps := $(smala_objs:.o=.d)
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

clean:
	rm -rf build
.PHONY: clean


