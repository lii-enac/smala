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


objs_cookbook_app :=  src/model/RectModel.o src/model/ModelManager.o \
 					src/view_model/RectViewModel.o src/view_model/ViewModelManager.o \
					src/text_view/TextView.o src/text_view/TextsListView.o \
					src/rect_view/Handle.o src/rect_view/RectView.o src/rect_view/RectanglesListView.o \
					src/main.o
djnn_libs_cookbook_app := gui display base exec_env core
smala_libs_cookbook_app := smala
