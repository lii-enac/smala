/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*/

use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton

import RectModel
import GraphicsController
import GraphicsDisplayView
import GraphicsPickingView
import TextController
import TextView
import LifetimeManager


_main_
Component root {
  Frame f ("mdpc", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  f.background_color.r = 255
  f.background_color.g = 255
  f.background_color.b = 255

  mouseTracking = 1 // for wheel events


  Component toolbox {
    Translation pos_buttons(10,0)
    StandAlonePushButton del ("Delete last", 0, 0)
    StandAlonePushButton add ("Add Rectangle", 0, 0)
    f.height - 40 =:> pos_buttons.ty
    add.x + add.width + 10 =:> del.x
  }


  List lifetime_managers
  List models
  List text_views

  // // transform the views
  // Translation _(45, 100)
  // Scaling _(0.5, 0.5, 0, 0)
  // Rotation _(22.5, 0, 0)

  List views

  Int text_y(15)
  toolbox.add.click -> (root) {
    Process model = ModelRect (root.models, "", 50, 50, 100, 70)
    Process lifetime_manager = LifetimeManager (root.lifetime_managers, "", model)

    Process gdv = GraphicsDisplayView (root.views, "")
    Process gpv = GraphicsPickingView (root.views, "")
    GraphicsController (lifetime_manager.controllers, "", model, gdv, gpv, root.f)

    Process tv = TextView (root.text_views, "", $root.text_y)
    TextController (lifetime_manager.controllers, "", model, tv)
    root.text_y += 15
  }

  toolbox.del.click -> del_action:(root) {
    if (root.lifetime_managers.size > 0) {
      int sz = root.lifetime_managers.size
      notify root.lifetime_managers.[sz].about_to_delete
    }
  }
}
