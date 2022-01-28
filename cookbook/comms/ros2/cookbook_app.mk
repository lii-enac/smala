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

objs_cookbook_app := ros_publisher.o ros_subscriber.o TextLineEdit.o main.o
djnn_libs_cookbook_app := gui display base exec_env core comms
smala_libs_cookbook_app := smala
cflags_cookbook_app += -I/opt/ros/galactic/include -I./cookbook/comms/ros2
# libs_cookbook_app += -L/opt/ros/galactic/lib -lrclcpp -lrcpputils -lrcl -lrcutils \
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
ros_libs_install_path ?= /opt/ros/galactic/lib

ros_libs := $(shell ls $(ros_libs_install_path)/lib*.so | xargs echo)
ros_libs := $(filter-out $(ros_libs_install_path)/librmw_cyclonedds_cpp.so, $(ros_libs))
ros_libs := $(patsubst $(ros_libs_install_path)/lib%.so,-l%,$(ros_libs))

# @Mathieu Magnaudet ou Poirier: to test: commentez ceci ^^ et decommentez cela vv  
# rlcpp_lib_deps := $(shell ldd $(ros_libs_install_path)/librlcpp.so | awk '{print $1}' | xargs echo | sed -e 's/.so.*//' | sed -e 's:/lib.*::'| sed -e 's/lib/-l/' | xargs echo)
# ros_libs := $(rlcpp_lib_deps)
# ros_libs := $(filter -l%,$(ros_libs))

libs_cookbook_app += -L$(ros_libs_install_path) $(ros_libs)
other_runtime_lib_path := $(ros_libs_install_path)

endif
