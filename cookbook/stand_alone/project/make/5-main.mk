#---- pkg-config -- 
# install with 
# 	make install in djnn-cpp and smala
# or with
#   package system : brew, apt, pacman ... 


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
djnn_cflags := -I$(djnn_cpp_path)/src
djnn_lib_path := $(djnn_cpp_path)/build/lib
djnn_libs ?= core exec_env base display comms gui input animation utils files audio
djnn_ldflags = -L$(djnn_lib_path) # $(addprefix -ldjnn-,$(djnn_libs))
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
CXXFLAGS += $(shell env PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(pkg_path) pkg-config --cflags $(pkg))
LIBS += $(shell env PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(pkg_path) pkg-config --libs $(pkg))
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


exe := $(exe)$(EXE)
exe := $(build_dir)/$(exe)

default app: $(exe)
.PHONY: default app

ld_library_path:=$(ld_library_path):$(abspath $(djnn_lib_path)):$(abspath $(smala_lib_path))

test: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)")
test_g: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)" -m geoportail)
test_g_p: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)" -m geoportail -p http://proxy.recherche.enac.fr:3128)
test_o: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)" -m osm)
test_o_p: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)" -m osm -p http://proxy.recherche.enac.fr:3128)

# Beynes (default): 48.86109526727752 1.8933138875646296

# Esperces: 43.315313261816485 1.404974527891014
test_g_e: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)" -m geoportail -lat 43.315313261816485 -lon 1.404974527891014)
test_g_p_e: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)" -m geoportail -p http://proxy.recherche.enac.fr:3128 -lat 43.315313261816485 -lon 1.404974527891014)

# Caylus: 44.27432196595285 1.729783361205679
test_g_c: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)" -m geoportail -lat 44.27432196595285 -lon 1.729783361205679)
test_g_p_c: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(launch_cmd) "$(shell pwd)/$(exe)" -m geoportail -p http://proxy.recherche.enac.fr:3128 -lat 44.27432196595285 -lon 1.729783361205679)

dbg: $(exe)
	(cd $(exe_dir); env $(LD_LIBRARY_PATH)=$(ld_library_path):$$$(LD_LIBRARY_PATH) $(debugger) "$(shell pwd)/$(exe)")
.PHONY: test

LD  = $(CXX)

objs_sma := $(srcs_sma:.sma=.o)
objs_sma := $(addprefix $(build_dir)/,$(objs_sma))
objs_other := $(srcs_other:.cpp=.o)
objs_other := $(addprefix $(build_dir)/,$(objs_other))

objs += $(objs_sma) $(objs_other)

gensrcs := $(objs_sma:.o=.cpp)
#$(objs_sma): $(gensrcs) # this forces the right language to compile the generated sources, but it will rebuild all sma files


ifeq ($(cross_prefix),em)
app_libs := $(addsuffix .bc,$(addprefix $(djnn_lib_path)/libdjnn-,$(djnn_libs)))
else
app_libs := $(addprefix -ldjnn-,$(djnn_libs))
endif



# --

distclean clear:
	rm -rf build .ninja_log
clean:
	rm -f $(gensrcs) $(objs) $(deps)
.PHONY: clean clear distclean

foo:
	echo $(objs_other)

