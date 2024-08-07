# 	djnn Smala compiler
#
# 	The copyright holders for the contents of this file are:
# 		Ecole Nationale de l'Aviation Civile, France (2024)
#  	See file "license.terms" for the rights and conditions
#  	defined by copyright holders.
#
#
#  	Contributors:
#  		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
#  		Stephane Conversy <stephane.conversy@enac.fr>
#		Jérémie Garcia 	  <jeremie.garcia@enac.fr>

objs_cookbook_app := src/main.o
djnn_libs_cookbook_app := gui display base exec_env core comms
# cppflags_cookbook_app := 

ifeq ($(os),Linux)
other_runtime_lib_path := /usr/lib64/
endif