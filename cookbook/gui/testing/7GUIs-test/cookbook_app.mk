#   djnn Smala compiler
#
#   The copyright holders for the contents of this file are:
#     Ecole Nationale de l'Aviation Civile, France (2017-2026)
#   See file "license.terms" for the rights and conditions
#   defined by copyright holders.
#
#
#   Contributors:
#     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
#     Stephane Conversy <stephane.conversy@enac.fr>
#	  Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
#	  Mathieu Poirier <mathieu.poirier@enac.fr>
#


objs_cookbook_app := Counter_7GUIs.o TempConverter_7GUIs.o FlightBooker_7GUIs.o Timer_7GUIs.o \
	../../7GUIs/5-CRUD/PersonModel.o ../../7GUIs/5-CRUD/PersonView.o ../../7GUIs/5-CRUD/PersonsList.o CRUD_7GUIs.o \
	../../7GUIs/6-circle-drawer/action/UndoRedoManager.o ../../7GUIs/6-circle-drawer/action/CreateAction.o ../../7GUIs/6-circle-drawer/action/ChangeRadiusAction.o ../../7GUIs/6-circle-drawer/CircleItem.o ../../7GUIs/6-circle-drawer/CircleDrawer.o CircleDrawer_7GUIs.o \
	../../7GUIs/7-cells/spreadsheet/Spreadsheet.o Cells_7GUIs.o \
	../../../../cppautogui/src/cppautogui_macos.o \
	main.o

djnn_libs_cookbook_app := gui display base exec_env core

smala_libs_cookbook_app := smala

#cppautogui
cppflags_cookbook_app += -Icppautogui/include/
libs_cookbook_app := -framework ApplicationServices -framework CoreFoundation -framework ImageIO
