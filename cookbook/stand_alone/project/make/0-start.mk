redirect: srcs.mk default

.PHONY: redirect config first second

# remove builtin rules: speed up build process and help debug
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:
