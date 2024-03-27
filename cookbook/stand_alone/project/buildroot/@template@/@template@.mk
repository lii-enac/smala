@TEMPLATE@_VERSION = origin/@template_git_branch@
@TEMPLATE@_SITE = @template_git_url@
@TEMPLATE@_SITE_METHOD = git
@TEMPLATE@_DEPENDENCIES = smala

# This is the actual build recipe

define @TEMPLATE@_BUILD_CMDS
	# config.mk
    @echo "djnn_cpp_path ?= ../djnn-gl-azdo" > $(@D)/config.mk
    @echo "smala_path ?= ../smala-origin_master" >> $(@D)/config.mk
	@echo "build_dir ?= build" >> $(@D)/config.mk
	@echo "no_pch := yes" >> $(@D)/config.mk
	
	$(MAKE) CC="$(TARGET_CC)" CXX="$(TARGET_CXX)" CXXFLAGS="$(HOST_CXXFLAGS)" PKG_CONFIG="$(PKG_CONFIG_HOST_BINARY)" -C $(@D) V=max
endef

# install recipe
define @TEMPLATE@_INSTALL_TARGET_CMDS
	#Volta
	$(INSTALL) -D -m 755 $(@D)/build/@template@ $(TARGET_DIR)/root/.
	# don't forget the ressources
	cp -r  $(@D)/res $(TARGET_DIR)/root/res
	$(INSTALL) -D -m 755 $(@D)/../../../package/@template@/launch_@template@.sh $(TARGET_DIR)/root/.
	$(INSTALL) -D -m 755 $(@D)/../../../package/@template@/ka $(TARGET_DIR)/usr/bin/.	
endef


$(eval $(generic-package))

#note 
# all variable : $(TARGET_CONFIGURE_OPTS) et *_FOR_BUILD = HOST_* or read LII_README
