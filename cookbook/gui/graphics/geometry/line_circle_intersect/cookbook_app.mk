# 	djnn Smala compiler
#
# 	The copyright holders for the contents of this file are:
# 		Ecole Nationale de l'Aviation Civile, France (2017)
#  	See file "license.terms" for the rights and conditions
#  	defined by copyright holders.
#
#
#  	Contributors:
#  		Stephane Conversy <stephane.conversy@enac.fr>
#

objs_cookbook_app := main.o
djnn_libs_cookbook_app := gui display base exec_env core
#pkg_cookbook_app := opencv

cflags_cookbook_app += -I$(app_srcs_dir)

ifeq ($(os),Darwin)
endif

ifeq ($(os),Linux)
endif

