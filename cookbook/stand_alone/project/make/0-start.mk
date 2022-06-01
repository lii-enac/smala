redirect: config.mk srcs.mk default
.PHONY: redirect

# remove builtin rules: speed up build process and help debug
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:
