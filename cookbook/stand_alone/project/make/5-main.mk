#---- pkg-config -- 
# install with 
# 	make install in djnn-cpp and smala
# or with
#   package system : brew, apt, pacman ... 


ifneq ($(djnn-pkgconf),)
pkge := $(shell pkg-config $(djnn-pkgconf) --exists; echo $$?)
ifeq ($(pkge),0)
djnn_cflags := $(shell pkg-config $(djnn-pkgconf) --cflags)
djnn_ldflags := $(shell pkg-config $(djnn-pkgconf) --libs)
djnn_lib_path := $(shell pkg-config $(djnn-pkgconf) --libs-only-L)
djnn_lib_path := $(subst -L, , $(djnn_lib_path))
endif
# undefine pkge
endif

ifneq ($(smala-pkgconf),)
#smalac := "should be in /usr/(local)/bin"
pkge := $(shell pkg-config $(smala-pkgconf) --exists; echo $$?)
ifeq ($(pkge),0)
smala_cflags := $(shell pkg-config $(smala-pkgconf) --cflags)
smala_ldflags := $(shell pkg-config $(smala-pkgconf) --libs)
smala_lib_path := $(shell pkg-config $(smala-pkgconf) --libs-only-L)
smala_lib_path := $(subst -L, , $(smala_lib_path))
endif
# undefine pkge
endif

#---- local sources

ifneq ($(djnn_cpp_path),)
djnn_cflags := -I$(djnn_cpp_path)/src
djnn_lib_path := $(djnn_cpp_path)/build/lib
djnn_modules ?= core exec_env base display comms gui input animation utils files audio
djnn_ldflags = -L$(djnn_lib_path)
endif

ifneq ($(smala_path),)
smalac := $(smala_path)/build/smalac
smala_cflags := -I$(smala_path)/build/src_lib
smala_ldflags := -L$(smala_path)/build/lib -lsmala
smala_lib_path := $(smala_path)/build/lib
smala_lib_dir ?= $(smala_path)/build/lib
endif

# for emscripten
em_ext_libs_path := ../../../djnn-emscripten-ext-libs


# ---------------------------------------
# save user-provided CXXFLAGS, and use CXXFLAGS as the ultimate compiler configuration

CXXFLAGS_CFG := $(CXXFLAGS)
CXXFLAGS :=

# by default, generate dependency .d files
CXXFLAGS += -MMD

# cross-compile support
ifndef cross_prefix
cross_prefix := g
#cross_prefix := em
#options: g llvm-g i686-w64-mingw32- arm-none-eabi- em
#/Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-c
#/usr/local/Cellar/android-ndk/r14/toolchains/arm-linux-androideabi-4.9/prebuilt/darwin-x86_64/bin/arm-linux-androideabi-g
endif

CC ?= $(cross_prefix)cc
CXX ?= $(cross_prefix)++


linker ?= $(compiler)

ifeq ($(linker),mold)
# ifeq ($(os),Darwin)
# CXXLD := ld64.mold
# CXXFLAGS += -fPIC
# LDFLAGS += -L/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/lib/
# LDFLAGS += -dylib -lc++ -lc
ifeq ($(compiler),gcc)
CXXLD ?= $(CXX) --use-ld=mold
endif
ifeq ($(compiler),llvm)
CXXLD ?= $(CXX) -fuse-ld=mold
endif

endif

CXXLD ?= $(CXX)

ifneq ($(pkg),)
#$1_lib_pkgpath = $$(subst $$() $$(),:,$$(lib_pkgpath))
CXXFLAGS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(pkg_path) pkg-config --cflags $(pkg))
LIBS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(pkg_path) pkg-config --libs $(pkg))
endif


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

original_exe := $(exe)
exe := $(exe)$(EXE)
exe := $(build_dir)/$(exe)

default app: config_test $(exe)
#.PHONY: default app

ld_library_path := $(call join-with,:,$(ld_library_path))
ld_library_path := $(ld_library_path):$(abspath $(djnn_lib_path)):$(abspath $(smala_lib_path))

config_test:
	@if [ ! -f config.mk ]; then \
		printf "\033[0;31m \n\n The config.mk file does not exist. Please run 'make config' to create it and configure it.\033[0m\n\n" ; exit 1 ; \
	fi

test: config_test $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)")
dbg: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(debugger) "$(shell pwd)/$(exe)")
.PHONY: test config_test

LD  = $(CXX)

objs_sma := $(srcs_sma:.sma=.o)
objs_sma := $(addprefix $(build_dir)/,$(objs_sma))
srcs_other_cpp := $(filter %.cpp,$(srcs_other))
srcs_other_c := $(filter %.c,$(srcs_other))
objs_other := $(srcs_other_cpp:.cpp=.o) $(srcs_other_c:.c=.o)
objs_other := $(addprefix $(build_dir)/,$(objs_other))

objs += $(objs_sma) $(objs_other)

gen_srcs := $(objs_sma:.o=.cpp)
#$(objs_sma): $(gen_srcs) # this forces the right language to compile the generated sources, but it will rebuild all sma files


ifeq ($(cross_prefix),em)
app_libs := $(addsuffix .bc,$(addprefix $(djnn_lib_path)/libdjnn-,$(djnn_modules)))
else
app_libs := $(addprefix -ldjnn-,$(djnn_modules))
#app_libs := $(addsuffix .a,$(addprefix $(djnn_lib_path)/libdjnn-,$(djnn_modules)))
#app_libs := $(shell PKG_CONFIG_PATH=$(djnn_lib_path)/.. pkg-config --libs --static djnn-cpp)
endif



# --

distclean clear:
	rm -rf build .ninja_log
clean:
	rm -f $(gen_srcs) $(objs) $(deps)
.PHONY: clean clear distclean
