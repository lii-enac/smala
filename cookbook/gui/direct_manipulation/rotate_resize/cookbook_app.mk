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
#		Jérémie Garcia 	  <jeremie.garcia@enac.fr>

objs_cookbook_app := src/LocalToLocal.o src/Drag.o src/HandleN.o src/HandleNE.o src/HandleE.o src/HandleSE.o src/HandleS.o src/HandleSW.o src/HandleW.o src/HandleNW.o src/RotationHandle.o src/RotateResizeRectangle.o src/main.o
djnn_libs_cookbook_app := gui display base core display
