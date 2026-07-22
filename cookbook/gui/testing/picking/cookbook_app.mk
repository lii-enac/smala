#   djnn Smala compiler
#
#   The copyright holders for the contents of this file are:
#     Ecole Nationale de l'Aviation Civile, France (2017-2026)
#   See file "license.terms" for the rights and conditions
#   defined by copyright holders.
#
#
#   Contributors:
#     Stephane Conversy <stephane.conversy@enac.fr>
#	  Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
#	  Mathieu Poirier <mathieu.poirier@enac.fr>
#


objs_cookbook_app := ../../../../cppautogui/src/cppautogui_macos.o \
	main.o

djnn_libs_cookbook_app := gui display base exec_env core

smala_libs_cookbook_app := smala

#cppautogui
cppflags_cookbook_app += -Icppautogui/include/
libs_cookbook_app := -framework ApplicationServices -framework CoreFoundation -framework ImageIO
