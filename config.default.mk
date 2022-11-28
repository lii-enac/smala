build_dir := build


#devel
#djnn-pkgconf = djnn-cpp-dev
#install (github)
djnn-pkgconf = djnn-cpp

#or use on local
djnn_path = ../djnn-cpp


cookbook_app_for_make_test := button
cookbook_apps_extra :=

SMAFLAGS += #-g #-gen-cleaner

# cross-compile support
#cross_prefix := em
#cookbook_cross_prefix := em

# emscripten ext libs
#em_ext_libs_path := ../djnn-emscripten-ext-libs

