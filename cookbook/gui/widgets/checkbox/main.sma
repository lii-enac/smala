/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
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

import gui.widgets.Label
import gui.widgets.CheckBox
import gui.widgets.RadioButton
import gui.widgets.VBox
import gui.widgets.HBox
import gui.widgets.CheckBoxManager

_main_
Component root {
  Frame f ("my frame", 500, 500, 300, 100)
  f.background_color.r = 200
  f.background_color.g = 200
  f.background_color.b = 200
  Exit ex (0, 1)
  f.close -> ex

    HBox hbox (f)
    Label l ("")
    l.preferred_height = 20
    l.preferred_width = 100
    VBox vbox (hbox)
      CheckBox c1 ("First choice")
      c1.h_alignment = 0
      CheckBox c2 ("Second choice")
      c2.h_alignment = 0
      CheckBox c3 ("Third choice")
      c3.h_alignment = 0
      addChildrenTo vbox.items {
        c1, 
        c2,
        c3
      }
      CheckBoxManager manager (vbox.items, vbox.items.[1])
    manager.value =:> l.text
    addChildrenTo hbox.items {
      l, vbox
    }
}