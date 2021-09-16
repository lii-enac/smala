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

import gui.widgets.CheckBox
import gui.widgets.RadioButton
import gui.widgets.WidgetContainer
import gui.widgets.VBox
import gui.widgets.CheckBoxManager

_main_
Component root {
  Frame f ("my frame", 0, 0, 400, 150)
  Exit ex (0, 1)
  f.close -> ex
  TextPrinter tp
  WidgetContainer container (f) {
    VBox vbox (0)
    CheckBox c1 (container, "First choice", 0, 0)
    CheckBox c2 (container, "Second choice", 0, 0)
    CheckBox c3 (container, "Third choice", 0, 0)
    addChildrenTo vbox.items {
      c1, 
      c2,
      c3
    }
    CheckBoxManager manager (vbox.items, vbox.items.[1])
    TextPrinter tp
    manager.value =:> tp.input
  }
}