build_dir := build


#devel
#djnn-pkgconf = djnn-cpp-dev
#install (github)
djnn-pkgconf = djnn-cpp

#or use on local
djnn_path = ../djnn-cpp


cookbook_app_for_make_test := button
cookbook_apps_extra :=

#smala flags
#SMAFLAGS += -g #-gen-cleaner

#C++ flags 
#CFLAGS_COMMON += -g
#LDFLAGS_COMMON +=

#Sanitizer 
#CFLAGS_COMMON += -fsanitize=thread -O1
#LDFLAGS_COMMON += -fsanitize=thread
#CFLAGS_COMMON += -fsanitize=address -O1
#LDFLAGS_COMMON += -fsanitize=address
#CFLAGS_COMMON += -fsanitize=memory -O1
#LDFLAGS_COMMON += -fsanitize=memory

# cross-compile support
#cross_prefix := em
#cookbook_cross_prefix := em

# emscripten ext libs
#em_ext_libs_path := ../djnn-emscripten-ext-libs

