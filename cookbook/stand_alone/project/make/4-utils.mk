
# uniq https://stackoverflow.com/a/16151140
uniq = $(if $1,$(firstword $1) $(call uniq,$(filter-out $(firstword $1),$1)))

# recursive wildcard https://stackoverflow.com/a/12959694
rwildcard = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
rwildcardmul = $(wildcard $(addsuffix $2, $1)) $(foreach d,$(wildcard $(addsuffix *, $1)),$(call rwildcard,$d/,$2))

# join https://stackoverflow.com/a/9551487
# space :=
# space +=
# join-with = $(subst $(space),$1,$(strip $2))
join-with = $(subst $(eval) ,$1,$(strip $2))


# ---------------------------------------
# stylized actions

# Set silent_opt to 's' if --quiet/-s set, otherwise ''.
silent_opt := $(filter s,$(word 1, $(MAKEFLAGS)))

silent =
ifeq ($(silent_opt),s)
silent = yes
endif
ifeq ($V,0)
silent = yes
endif

ifeq ($(silent),yes)
rule_message =
else
rule_message = printf "$1 $2\n"
endif

ifeq ($V,)
stylized_target = "\\033[1m$(notdir $@)\\033[0m"
else ifeq ($V,0)
#stylized_target = "\\033[1m$(notdir $@)\\033[0m"
else ifeq ($V,1)
stylized_target = "\\033[1m$(notdir $@)\\033[0m"
else ifeq ($V,2)
stylized_target = "\\033[1m$(notdir $@)\\033[0m" (older than $(notdir $?))
else ifeq ($V,3)
stylized_target = "\\033[1m$@\\033[0m" (older than $?)
endif
#in case it doesn't work on some terminals:
#stylized_target = $(notdir $@) \(older than $(notdir $?)\)


# ---------------------------------------
# generate ninja file from Makefile

ninja: build.ninja
build.ninja: make2ninja.py
	make -nd V=max test | python3 make2ninja.py > build.ninja
.PHONY: build.ninja ninja

make2ninja.py:
	curl -O https://raw.githubusercontent.com/conversy/make2ninja/90e939a539a081b44c9bdc2bef70e6908e7a645a/make2ninja.py
ifeq ($(os),Darwin)
	sed -i '' "s/\'(./\`(./" make2ninja.py
endif
