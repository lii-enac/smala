/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use gui

import Button


_action_
myFunc (Component c) 
%{
  printf ("my code\n");
%}


_main_
Component root {
  Frame f ("my frame", 0, 0, 400, 600)
  Exit ex (0, 1)
  f.close -> ex
  svg = loadFromXML("file:///Users/magnaudet/Dev/smala/volta-hmi-graphic-design_6.svg")
  g << svg.layer1
}

run root
run syshook
