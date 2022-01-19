/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2019)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Stéphane Conversy <stephane.conversy@enac.fr>
 *
 */

use core
use base
use display
use gui

import gui.widgets.Slider

// does not work yet, but it would be excellent it if would
// idea: when detecting a cycle, test if it's with properties
// if yes
// test previous value: if it's the same, silently ignore the cycle
// if it's not the same, raise the error

_main_
Component root {
  _DEBUG_GRAPH_CYCLE_DETECT = 1

  Frame f ("my frame", 0, 0, 1200, 600)
  Exit ex (0, 1)
  f.close -> ex

  // conversion model (bidi)

  Double fahrenheit (0)
  Double celsius (0)

  (5./9.) * (fahrenheit-32) => celsius
  (9./5.) * celsius + 32 => fahrenheit

  // GUI
  FillColor _(255,255,255)
  Text tf(10, 10, "fahrenheit")
  Slider sf (f, 10, 10)
  Text tc(10, 100, "celsius")
  Slider sc (f, 10, 100)

  // bidi
  fahrenheit => sf.value
  sf.value => fahrenheit

  // bidi
  celsius => sc.value
  sc.value => celsius
}
