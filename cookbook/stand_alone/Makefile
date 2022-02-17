# canonical Makefile for smala applications
# 1. copy stand_alone directory somwhere: cp -r cookbook/stand_alone /some/where/else
# 2. edit configuration part (executable name, srcs, djnn_libs, path to djnn-cpp and smalac)
# 3. make test

# you should not edit this file

redirect: config.mk default
.PHONY: redirect

project_dir := project/make

config.mk:
	cp $(project_dir)/1-config.default.mk config.mk

# include user-specified config if present
-include config.mk
# default config
include $(project_dir)/1-config.default.mk
# sources information
include srcs.mk
# main stuff
include $(project_dir)/0-main.mk

