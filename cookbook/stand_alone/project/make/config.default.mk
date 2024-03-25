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

# DJNN_CXXFLAGS += -Wall
# DJNN_CXXFLAGS += -g
# DJNN_CXXFLAGS += -O0
# SMAFLAGS += -g
# SMAFLAGS += -fastcomp

# DJNN_CXXFLAGS += -fsanitize=address
# LDFLAGS += -fsanitize=address
# DJNN_CXXFLAGS += -fsanitize=thread
# LDFLAGS += -fsanitize=thread

# CXX := clang++-14
# compiler := llvm
# linker := mold

# keep_intermediate := yes  # yes | no
# use_deps := yes  # yes | no
