# 	djnn Smala compiler
#
# 	The copyright holders for the contents of this file are:
# 		Ecole Nationale de l'Aviation Civile, France (2017-2018)
#  	See file "license.terms" for the rights and conditions
#  	defined by copyright holders.
#
#
#  	Contributors:
#  		Stephane Conversy <stephane.conversy@enac.fr>

objs_cookbook_app := src/midi.o src/midi_p.o src/main.o
djnn_libs_cookbook_app := gui display base exec_env core comms
smala_libs_cookbook_app := smala

ifeq ($(os),Linux)
cppflags_cookbook_app := -I/usr/include/RtMidi
endif

libs_cookbook_app := -lrtmidi

