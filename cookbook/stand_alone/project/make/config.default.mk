build_dir ?= build

# release and installed (brew, apt, pacman)
#djnn-pkgconf ?= djnn-cpp
#smala-pkgconf ?= smala
# devel version : djnn-cpp-dev and smala-dev

# or local sources, uncomment _all_ following lines
djnn-pkgconf :=
smala-pkgconf :=
djnn_cpp_path ?= ../djnn-cpp
smala_path ?= ../smala

#CXXFLAGS += -DNO_ROS -DNO_LEMON

SMAFLAGS += -g #-gen-cleaner
CXXFLAGS += -g #-Wall

#CXXFLAGS += -fsanitize=address -O1
#LDFLAGS += -fsanitize=address
#CXXFLAGS += -fsanitize=thread -O1
#LDFLAGS += -fsanitize=thread
