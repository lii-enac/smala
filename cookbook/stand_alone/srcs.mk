#relative path
src_dir := src
res_dir := res
exe_dir := .

exe := button

srcs_sma := src/Button.sma src/main.sma
# or
#srcs_sma := $(shell find $(src_dir) -name "*.sma")

# native sources
srcs_other := src/foo.cpp
# or
#srcs_other := $(shell find $(src_dir) -name "*.cpp")

# external libraries
# CXXFLAGS += $(shell pkg-config --cflags foo)
# LDFLAGS +=
# LIBS += $(shell pkg-config --libs foo)
# or
# pkgdeps += foo
