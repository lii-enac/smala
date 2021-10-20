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
use display
use gui

import DraggableCircle
import CollisionFilter

_main_
Component root {
  Frame f ("my frame", 0, 0, 400, 600)
  Exit ex (0, 1)
  f.close -> ex

  Ref selected (0)
  List obj {
    DraggableCircle _ ( 50, 50, 10)
    DraggableCircle _ ( 70, 50, 10)
    DraggableCircle _ ( 50, 70, 10)
    DraggableCircle _ ( 90, 50, 10)
    DraggableCircle _ ( 50, 90, 10)
  }
  for item:obj {
    item.press->{item =: selected}
  }
  CollisionFilter filter (obj, selected)
}
