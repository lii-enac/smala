#relative path
src_dir ?= src
res_dir ?= res
exe_dir ?= .

exe ?= stand_alone

# smala sources
srcs_sma := $(shell find $(src_dir) -name "*.sma")
# or 
# srcs_sma ?= src/main.sma

# native sources
srcs_other := $(shell find $(src_dir) -name "*.cpp")
# or
# srcs_other ?=

# external libraries
# pkgdeps += curl
# or
# CXXFLAGS += $(shell pkg-config --cflags foo)

srcs.mk:
	cp $(project_dir)/3-srcs.default.mk srcs.mk
