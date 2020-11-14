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

objs_cookbook_app := main.o
djnn_libs_cookbook_app := gui display base exec_env core
lang_cookbook_app := cpp
smala_libs_cookbook_app := smala
pkg_cookbook_app := libcurl
cflags_cookbook_app += -I$(app_srcs_dir)