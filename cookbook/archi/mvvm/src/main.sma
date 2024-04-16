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
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/

use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton

import ViewModelManager

import view.TextsListView
import view.RectanglesListView

// import GraphicsController
// import GraphicsView
// import TextController
// import TextView
// import TextViewModel
// import LifetimeManager


_main_
Component root {
  Frame f ("mvvm", 0, 0, 600, 600)
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

  Int texts_list_width (250)

  ViewModelManager VM_manager ()

  TextsListView texts_list_view (VM_manager)
  texts_list_width =: texts_list_view.width

  RectanglesListView rectangles_list_view (VM_manager)
  texts_list_width + 4 =: rectangles_list_view.x
  f.width - rectangles_list_view.x =:> rectangles_list_view.width

  f.height =:> texts_list_view.height, rectangles_list_view.height


  // List lifetime_managers
  // List models
  // List view_models
  // List views

  // Int text_y(15)
  // toolbox.add.click -> (root) {
  //   Process model = RectModel (root.models, "", 50, 50, 100, 70)
  //   Process view_model = RectViewModel (root.view_models, "", model)

  //   Process lifetime_manager = LifetimeManager (root.lifetime_managers, "", model)

  //   Process gv = GraphicsView (root.views, "")
  //   GraphicsController (lifetime_manager.controllers, "", model, gv, root.f)

  //   Process tv = TextView (root.views, "", $root.text_y)
  //   Process tvm = TextViewModel (root.view_models, "", tv)
  //   TextController (lifetime_manager.controllers, "", model, tv, tvm)
  //   root.text_y += 15
  // }
  toolbox.add.click -> VM_manager.new_rectangle

  // toolbox.del.click -> del_action:(root) {
  //   if (root.lifetime_managers.size > 0) {
  //     int sz = root.lifetime_managers.size
  //     notify root.lifetime_managers.[sz].about_to_delete
  //   }
  // }
  toolbox.del.click -> VM_manager.delete_rectangle

}
