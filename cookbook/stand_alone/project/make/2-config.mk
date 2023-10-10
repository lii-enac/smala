# include user-specified config if present
-include config.mk

# default config, which should only use ?= (set a variable if it has not been set by the user-modified config.mk), because we don't want to replace user's configuration
include $(project_dir)/config/config_default.mk

config:
	cp -n $(project_dir)/config/config_default.mk config.mk

config_%:
	cp $(project_dir)/config/$@.mk config.mk

.PHONY: config
