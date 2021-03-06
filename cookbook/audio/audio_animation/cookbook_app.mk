# 	djnn Smala compiler
#
# 	The copyright holders for the contents of this file are:
# 		Ecole Nationale de l'Aviation Civile, France (2017)
#  	See file "license.terms" for the rights and conditions
#  	defined by copyright holders.
#
#
#  	Contributors:
#  		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
#  		Stephane Conversy <stephane.conversy@enac.fr>
#

objs_cookbook_app := audio_animation.o

ifeq ($(os),Linux)
libs_cookbook_app := -lopenal
endif

djnn_libs_cookbook_app := animation gui display audio base exec_env core
smala_libs_cookbook_app := smala
res_dir := ../simple_audio/res
