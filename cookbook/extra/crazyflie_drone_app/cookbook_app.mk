# usage :
#
# add this to config.mk in djnn-cpp/:
# crazyflie_firmware_dir := ../../apps/crazyflie-push-demo/crazyflie-firmware
# #include src/exec_env/crazyflie/not-a-djnn-lib.mk
# djnn_libs := core base exec_env
# os := crazyflie
# cross_prefix := arm-none-eabi-
# display :=
# graphics :=
#
# then in djnn-cpp/:
# make -j static
#
# add this to config.mk in smala/:
# cookbook_apps_extra := extra/crazyflie_drone_app
# cookbook_app_for_make_test := crazyflie_drone_app
# cookbook_cross_prefix := arm-none-eabi-
# crazyflie_firmware_dir := ../../apps/crazyflie-push-demo/crazyflie-firmware
#
# then in smala/:
# make -j crazyflie_drone_app_objs
#
# clone https://github.com/ataffanel/crazyflie-push-demo
# apply the diff in crazyflie-push-demo.diff
# then:
# make -j && make cload
#
# git diff fabc98587e82a0408057247758899afe5902275f > diff
# git diff fabc98587e82a0408057247758899afe5902275f > ../../djnn/smala/cookbook/extra/crazyflie_drone_app/crazyflie-push-demo.diff

objs_cookbook_app := crazyflie_drone_app.o #fake_main.o
djnn_libs_cookbook_app := crazyflie
#smala_libs_cookbook_app := smala

# freertos
freertos_dir := $(crazyflie_firmware_dir)/vendor/FreeRTOS
freertos_config_dir := $(crazyflie_firmware_dir)/src/config
freertos_layers_dir := $(crazyflie_firmware_dir)/src

CFLAGS += -I$(freertos_dir)/include \
	-I$(freertos_dir)/portable/GCC/ARM_CM4F \
	-I$(freertos_config_dir) \
	-I$(freertos_layers_dir)/drivers/interface \
	-I$(freertos_layers_dir)/hal/interface \
	-I$(freertos_layers_dir)/utils/interface \
	-I$(freertos_layers_dir)/modules/interface

#freetos-cxx11
CXXFLAGS += -include $(djnn_path)/src/exec_env/freertos/ext/freertos-cxx11/freertos-cxx11-macros.h
CXXFLAGS += -I$(djnn_path)/src/exec_env/freertos/ext/freertos-cxx11

# crazyflie
CFLAGS += -DSTM32F40_41xxx
CFLAGS += -I$(crazyflie_firmware_dir)/src/lib/CMSIS/STM32F4xx/Include
CFLAGS += -I$(crazyflie_firmware_dir)/vendor/CMSIS/CMSIS/Include/
CFLAGS += -mfp16-format=ieee -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
#CFLAGS += -pie
CFLAGS += -fexceptions
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-math-errno -fno-strict-aliasing -Wdouble-promotion
#CFLAGS += -fpic

#boost
CXXFLAGS += -I/usr/local/include

# djnn
CFLAGS += -DDJNN_CRAZYFLIE
CFLAGS += -DDJNN_NO_DEBUG
CFLAGS += -DDJNN_NO_SERIALIZE
CFLAGS += -DRMT_ENABLED=0
#CXXFLAGS += $(CFLAGS)
#CXXFLAGS += -DDJNN_NO_DYNAMIC_CAST
CXXFLAGS += -DDJNN_USE_FREERTOS
#CXXFLAGS += -DDJNN_USE_FREERTOS_MAINLOOP
CXXFLAGS += -DDJNN_USE_STD_THREAD=1
CXXFLAGS += --rtti #--rtti_data
CXXFLAGS += -Wno-psabi #https://stackoverflow.com/a/48149400
