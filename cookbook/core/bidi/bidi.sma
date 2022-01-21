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

import HMI

// does not work yet, but it would be excellent it if would
// idea: when detecting a cycle, test if it's with properties
// if yes
// test previous value: if it's the same, silently ignore the cycle
// if it's not the same, raise the error

_main_
Component root {
  _DEBUG_GRAPH_CYCLE_DETECT = 1

  Double fahrenheit (0)
  Double celsius (0)

  (5./9.) * (fahrenheit-32) => celsius
  (9./5.) * celsius + 32 => fahrenheit

  HMI hmi()
  p aka hmi.wc.vbox

  fahrenheit => p.sf.value
  p.sf.value => fahrenheit
  
  celsius => p.sc.value
  p.sc.value => celsius
}
