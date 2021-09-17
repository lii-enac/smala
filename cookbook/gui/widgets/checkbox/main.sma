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
import gui.widgets.WidgetContainer
import gui.widgets.VBox
import gui.widgets.HBox
import gui.widgets.CheckBoxManager

_main_
Component root {
  Frame f ("my frame", 500, 500, 300, 100)
  Exit ex (0, 1)
  f.close -> ex

  WidgetContainer container (f) {
    HBox hbox (0)
    Label l (container, "", 0, 0)
    WidgetContainer wc (container) {
      VBox vbox (1)
      CheckBox c1 (wc, "First choice", 0, 0)
      CheckBox c2 (wc, "Second choice", 0, 0)
      CheckBox c3 (wc, "Third choice", 0, 0)
      addChildrenTo vbox.items {
        c1, 
        c2,
        c3
      }
      CheckBoxManager manager (vbox.items, vbox.items.[1])
    }
    wc.manager.value =:> l.text
    addChildrenTo hbox.items {
      l, wc
    }
  }
}