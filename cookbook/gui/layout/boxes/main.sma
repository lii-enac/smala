/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2021)
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

import gui.widgets.IWidget
import gui.widgets.Button
import gui.widgets.HBox
import gui.widgets.VBox

_main_
Component root {
  Frame f ("frame", 0, 0, 600, 200)
  Exit ex (0, 1)
  f.close->ex
  TextPrinter tp
  HBox vb1 (f, 0)
  Button b1 (f, "My Button", 0, 0)
  Button b2 (f, "Quit", 0, 0)
  b2.click->ex
  Button b3 (f, "Yet Another Button", 0, 0)
  b3.click -> { "Hello!" =: tp.input }

  IWidget sub (f)
  addChildrenTo sub {
    Translation pos (0, 0)
    x aka pos.tx
    y aka pos.ty
    NoFill _
    NoOutline _
    Rectangle r (0, 0, 0, 0, 0, 0)
    sub.width =:> r.width
    sub.height =:> r.height
    Button sub1 (r, "Sub button 1", 0, 0)
    Button sub2 (r, "Sub button 2", 0, 0)
    Button sub3 (r, "Sub button 3", 0, 0)
    VBox hbox (sub, 1)
    addChildrenTo hbox.items {
      << sub1
      << sub2
      << sub3
    }
  }

  addChildrenTo vb1.items {
    << b1
    << sub
    << b2
    << b3
  }
}
