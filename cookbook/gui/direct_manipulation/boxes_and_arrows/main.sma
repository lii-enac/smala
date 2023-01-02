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

import MyBox
import BoxesAndArrowsManager
import gui.widgets.StandAlonePushButton

_main_
Component root {
  Frame f ("my frame", 0, 0, 600, 400)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1
  f.background_color.r = 0
  f.background_color.g = 17
  f.background_color.b = 61

  Component gui {
    Component boxes
    Component arrows
    StandAlonePushButton button ("Add box", 0, 0)
    f.width - button.width =:> button.x
/*    NoOutline _
    FillColor _ (#AFAFAF)
    Rectangle plus (0, -2, 25, 25, 2, 2)
    f.width - 23 =:> plus.x
    OutlineColor _ (#101010)
    OutlineWidth _ (3)
    Translation pos (0, 0)
    Line l1 (12, 3, 12, 21)
    Line l2 (3, 12, 21, 12)
    plus.x =:> pos.tx */
  }
  BoxesAndArrowsManager manager (gui)

  Spike add_box
  Int cur_x (30)
  Int cur_y (3)
  add_box->(root) {
    addChildrenTo root.gui.boxes {
      MyBox _ (root.manager, $root.cur_x, $root.cur_y)
      root.cur_x += 5
      root.cur_y += 5
    }
  }
  gui.button.click->add_box
}
