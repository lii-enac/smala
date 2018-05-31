# canonical Makefile for smala applications

#exe := button
#srcs := Button.sma main.sma
#target_lang := c

exe := MyApp
java_package := truc
srcs := Button.sma MyApp.sma
target_lang := java

djnn_libs := gui base core

smalac := ../../build/smalac
djnn_path := ../../../local-install
djnn_java_classpath := ../../../djnn-java/src:../../../djnn-java/src/jna.jar
build_dir := build

# --
# hopefully no need to tweak the lines below

# remove builtin rules: speed up build process and help debug
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:


# -- c

ifeq ($(target_lang),c)

exe := $(build_dir)/$(exe)

default: $(exe)
.PHONY: default

test: $(exe)
	$(exe)
.PHONY: test

CFLAGS ?= -g

objs := $(srcs:.sma=.o)
objs := $(addprefix $(build_dir)/,$(objs))

$(objs): CFLAGS += -I$(djnn_path)/include -MMD
$(exe): LDFLAGS += -L$(djnn_path)/lib
$(exe): LIBS += $(addprefix -ldjnn-,$(djnn_libs))

$(exe): $(objs)
	@mkdir -p $(dir $@)
	$(CC) $^ -o $@ $(LDFLAGS) $(LIBS)

# for .c srcs if any
#$(build_dir)/%.o: %.c
#	@mkdir -p $(dir $@)
#	$(CC) $(CFLAGS) -c $< -o $@

# for generated .c from .sma
$(build_dir)/%.o: $(build_dir)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# .sma
$(build_dir)/%.c $(build_dir)/%.h: %.sma
	@mkdir -p $(dir $@)
	@echo $(smalac) $<
	@$(smalac) $< || (c=$$?; rm -f $*.c $*.h $*.java; (exit $$c))
	@if [ -f $*.c ]; then mv $*.c $(build_dir)/$(*D); fi;
	@if [ -f $*.h ]; then mv $*.h $(build_dir)/$(*D); fi;

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


