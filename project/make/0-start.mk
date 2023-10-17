redirect: default
.PHONY: redirect

# remove builtin rules: speed up build process and help debug
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:

MAJOR = 1
MINOR = 20
MINOR2 = 0

default: smalac
# config.mk # does not work anymore
.PHONY: default

all: smalac smala_lib cookbook_apps test_apps
# config.mk # does not work anymore
.PHONY: all

help:
	@echo "default: smalac ; all: smalac cookbook"
	@echo "button: will build button cookbook app (works with all cookbook apps: $(cookbook_apps))"
	@echo "button_test: will build button cookbook app and launch it (works with all cookbook apps: $(cookbook_apps))"
	@echo "experiment make -j !!"
