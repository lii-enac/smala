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
#

objs_cookbook_app := src/ControlManager.o src/Controller1.o src/Controller2.o src/View1.o src/View2.o src/Model.o src/main.o
djnn_libs_cookbook_app := gui display base exec_env core
smala_libs_cookbook_app := smala
