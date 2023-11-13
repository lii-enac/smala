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
*/

use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton
import Model
import View1
import View2

_main_
Component root {
  Frame f ("mvc", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1
  f.background_color.r = 255
  f.background_color.g = 255
  f.background_color.b = 255

  FillColor _ (Red)
  List models
  List views

  Translation pos_buttons(10,0)
  StandAlonePushButton del ("Delete last", 0, 0)
  StandAlonePushButton add ("Add Rectangle", 0, 0)
  f.height - 40 =:> pos_buttons.ty
  add.x + add.width + 10 =:> del.x

  Int ty(15)


  del.click -> (root) {
    if (root.models.size > 0) {
      int sz = root.models.size
      notify root.models.[sz].about_to_delete
    }
  }
  add.click -> (root) {
    Process model = Model (root.models, "", 50, 50, 100, 70)
    Process v1 = View1 (root.views, "", model) 
    Process v2 = View2 (root.views, "", model, $root.ty)
    root.ty += 15
    addChildrenTo model.views {
      Ref _(v1)
      Ref _(v2)
    }
  }


}
