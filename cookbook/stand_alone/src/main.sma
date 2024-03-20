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
use display
use gui

// local version
import Button

// smala lib
//import gui.widgets.Button

_native_code_
%{
#include <iostream>
%}

_action_
myFunc (Process c) 
%{
  std::cout << ("Button clicked") << std::endl;
%}


_main_
Component root {
  Frame f ("helloworld", 0, 0, 400, 600)
  f.background_color.r = 125
  f.background_color.g = 125
  f.background_color.b = 125
  Exit ex (0, 1)
  f.close -> ex

  NativeAction na (myFunc, 1)

  Button b (f, "Click", 10, 100)
  b.click -> na

  mouseTracking = 1
  FillColor _ (Yellow)
  Circle c (0, 0, 20)
  f.move.x => c.cx
  f.move.y => c.cy

  // loading a resource
  string path = "res/simple.svg"
  svg = load_from_XML (path)
  //dump svg
  Translation _(10,400)
  t << svg
}
