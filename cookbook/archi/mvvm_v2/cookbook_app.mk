#   djnn Smala compiler
#
#   The copyright holders for the contents of this file are:
#     Ecole Nationale de l'Aviation Civile, France (2017)
#   See file "license.terms" for the rights and conditions
#   defined by copyright holders.
#
#
#   Contributors:
#     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
#     Stephane Conversy <stephane.conversy@enac.fr>
#     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
#


objs_cookbook_app :=  src/model/RectModel.o src/ModelManager.o \
 					src/view_model/RectViewModel.o src/ViewModelManager.o \
					src/view/TextView.o src/view/TextsListView.o \
					src/view/Handle.o src/view/RectView.o src/view/RectanglesListView.o \
					src/main.o
djnn_libs_cookbook_app := gui display base exec_env core
smala_libs_cookbook_app := smala
