# 	djnn Smala compiler
#
# 	The copyright holders for the contents of this file are:
# 		Ecole Nationale de l'Aviation Civile, France (2017-2018)
#  	See file "license.terms" for the rights and conditions
#  	defined by copyright holders.
#
#
#  	Contributors:
#  		Stephane Conversy <stephane.conversy@enac.fr>

objs_cookbook_app := RosPublisher.o RosSubscriber.o TextLineEdit.o main.o
djnn_libs_cookbook_app := gui display base exec_env core comms
smala_libs_cookbook_app := smala
cflags_cookbook_app += -I/opt/ros/foxy/include
# libs_cookbook_app += -L/opt/ros/foxy/lib -lrclcpp -lrcpputils -lrcl -lrcutils \
# 	-lrmw -lrmw_implementation -lrcl_yaml_param_parser \
# 	-llibstatistics_collector -lrosgraph_msgs__rosidl_typesupport_cpp \
# 	-lstatistics_msgs__rosidl_typesupport_cpp \
# 	-ltracetools \
# 	-lrcl_interfaces__rosidl_typesupport_cpp \
# 	-lrcl_interfaces__rosidl_typesupport_c \
# 	-lrcl_logging_spdlog \
# 	-lrcl_interfaces__rosidl_generator_c \
# 	-lrosidl_typesupport_cpp \
# 	-lrosidl_typesupport_fastrtps_c \
# 	-lrosidl_typesupport_fastrtps_cpp \
# 	-lrosidl_typesupport_c \
# 	-lrosidl_runtime_c \
# 	-lbuiltin_interfaces__rosidl_generator_c \
# 	-laction_msgs__rosidl_typesupport_c \
# 	-laction_msgs__rosidl_typesupport_cpp \

ifeq ($(os),Linux)
ros_libs_install_path ?= /opt/ros/foxy/lib
ros_libs := $(shell ls $(ros_libs_install_path)/lib*.so | xargs echo)
ros_libs := $(patsubst $(ros_libs_install_path)/lib%.so,-l%,$(ros_libs))
endif

libs_cookbook_app += -L/opt/ros/foxy/lib $(ros_libs)

other_runtime_lib_path := /opt/ros/foxy/lib
