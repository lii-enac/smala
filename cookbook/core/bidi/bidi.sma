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

import gui.widgets.WidgetContainer
import gui.widgets.PushButton
import gui.widgets.VBox
import gui.widgets.HBox
import gui.widgets.Slider
import gui.widgets.Label

// does not work yet, but it would be excellent it if would
// idea: when detecting a cycle, test if it's with properties
// if yes
// test previous value: if it's the same, silently ignore the cycle
// if it's not the same, raise the error

_main_
Component root {
  //_DEBUG_GRAPH_CYCLE_DETECT = 1

  Frame f ("my frame", 0, 0, 300, 300)
  Exit ex (0, 1)
  f.close -> ex
  f.background_color.r = 200
  f.background_color.g = 200
  f.background_color.b = 200
  // conversion model (bidi)

  Double fahrenheit (0)
  Double celsius (0)

  (5./9.) * (fahrenheit-32) => celsius
  (9./5.) * celsius + 32 => fahrenheit

  // GUI
  WidgetContainer wc (f) {
    VBox hbox (0)
    Label l (wc, "farenheit:", 0, 0)
    Slider sf (wc, 0, 0)
    "farenheit: " + sf.value =:> l.text
    //bidi
    fahrenheit => sf.value
    sf.value => fahrenheit
    Label l2 (wc, "celsius:", 0, 0)
    Slider sc (wc, 0, 0)
    "celsius: " + sc.value =:> l2.text
    //bidi
    celsius => sc.value
    sc.value => celsius
    addChildrenTo hbox.items {l, sf, l2, sc}
  }
}
