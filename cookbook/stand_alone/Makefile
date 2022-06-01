# canonical Makefile for smala applications
# you should NOT edit this file, nor the files in project/make/*.mk !!
# instead:
# 1. copy stand_alone directory somewhere: cp -r cookbook/stand_alone /some/where/else
# 2. cd into it and launch a single 'make'
# 2. edit the configuration part in config.mk and srcs.mk (executable name, srcs, djnn_libs, path to djnn-cpp and smalac)
# 3. make test

project_dir := project/make

include $(project_dir)/0-start.mk

# determine host os
include $(project_dir)/1-os.mk

# include user-specified config if present
-include config.mk

# default config
include $(project_dir)/2-config.default.mk

# utils
include $(project_dir)/4-utils.mk

# sources
include srcs.mk

# main stuff
include $(project_dir)/5-main.mk

# generic rules
include $(project_dir)/7-pch.mk

# generic rules
include $(project_dir)/8-rules.mk

# pkg dependency
include $(project_dir)/9-pkgdeps.mk


