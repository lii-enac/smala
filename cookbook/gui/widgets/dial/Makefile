# 	djnn Smala compiler
#
# 	The copyright holders for the contents of this file are:
# 		Ecole Nationale de l'Aviation Civile, France (2017)
#  	See file "license.terms" for the rights and conditions
#  	defined by copyright holders.
#
#
#  	Contributors:
#  		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>

CC = gcc
BUILD_DIR = build
SMALAC = ../../build/smalac
DJNN_PATH = ../../../local-install

OBJ = src/Button.o src/ToggleButton.o src/Dial.o src/Slider.o src/main.o

CFLAGS = -g
INCDIR = $(DJNN_PATH)/include
LIBDIR = $(DJNN_PATH)/lib
LIBS = -ldjnn-core -ldjnn-base -ldjnn-gui -ldjnn-animation
COMPILE = $(CC) $(CFLAGS) -I$(INCDIR) -Isrc

all: $(OBJ)
	$(COMPILE) $(OBJ) -o app -L$(LIBDIR) $(LIBS)

src/%.o: src/%.c
	$(COMPILE) -c $< -o $@

src/%.c: src/%.sma
	$(SMALAC) $<

clean:
	rm src/*.o src/*.c src/*.h
	rm app
