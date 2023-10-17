build_dir ?= build

# release and installed (brew, apt, pacman)
djnn-pkgconf ?= djnn-cpp
smala-pkgconf ?= smala
# devel version : djnn-cpp-dev and smala-dev

# or local sources, uncomment _all_ following lines
# djnn-pkgconf :=
# smala-pkgconf :=
# djnn_cpp_path ?= ../djnn-cpp
# smala_path ?= ../smala

# CXX := clang++-14
# compiler := llvm
# linker := mold
# no_pch = yes

# CXXFLAGS += -O0
# CXXFLAGS += -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-vla-extension
# CXXFLAGS += -ftime-trace
# CXXFLAGS += -g
# CXXFLAGS += -fsanitize=address -O1
# LDFLAGS += -fsanitize=address
# CXXFLAGS += -fsanitize=thread -O1
# LDFLAGS += -fsanitize=thread

# SMAFLAGS += -gen-cleaner
# SMAFLAGS += -g
# SMAFLAGS += -fastcomp
# djnn_modules ?= core exec_env base display gui input animation utils files audio c_api
keep_intermediate ?= yes
# nodeps ?= 1
