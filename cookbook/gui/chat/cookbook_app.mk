# 	djnn Smala compiler
#
# 	The copyright holders for the contents of this file are:
# 		Ecole Nationale de l'Aviation Civile, France (2017-2020)
#  	See file "license.terms" for the rights and conditions
#  	defined by copyright holders.
#
#
#  	Contributors:
#  		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
#  		Stephane Conversy <stephane.conversy@enac.fr>
#		Mathieu Poirier <mathieu.poirier@enac.fr>
#

objs_cookbook_app := scrollbar/inverse_transform.o scrollbar/paging.o scrollbar/Scrollbar.o MyTextField.o Button.o  Bubble.o Chat.o main.o
djnn_libs_cookbook_app := utils gui display base exec_env core
smala_libs_cookbook_app := smala