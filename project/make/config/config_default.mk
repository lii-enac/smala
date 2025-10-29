#build_dir := build


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
#-g for text in debug_info

#C++ flags 
#CXXFLAGS_CK += -g
#LDFLAGS_CK +=

#Sanitizer only on the cookbook - CXXFLAGS_CK LDFLAGS_CK
#CXXFLAGS_CK += -fsanitize=thread -O1
#LDFLAGS_CK += -fsanitize=thread
#CXXFLAGS_CK += -fsanitize=address
#LDFLAGS_CK += -fsanitize=address
#CXXFLAGS_CK += -fsanitize=memory -O1
#LDFLAGS_CK += -fsanitize=memory

# cross-compile support
#cross_prefix := em
#cookbook_cross_prefix := em

# emscripten ext libs
#em_ext_libs_path := ../djnn-emscripten-ext-libs

#keep_intermediate = yes
