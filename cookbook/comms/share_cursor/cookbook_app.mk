# 	djnn Smala compiler
#
# 	The copyright holders for the contents of this file are:
# 		Ecole Nationale de l'Aviation Civile, France (2020-2021)
#  	See file "license.terms" for the rights and conditions
#  	defined by copyright holders.
#
#
#  	Contributors:
#  		Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
#

objs_cookbook_app := src/RemoteCursor.o src/ShareCursorPosition.o src/main.o
djnn_libs_cookbook_app := gui display base exec_env core comms

ifeq ($(os),Linux)
other_runtime_lib_path := /usr/lib64/
endif