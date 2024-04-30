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


_main_
Component root {
  Frame f ("mvvm", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  f.background_color.r = 255
  f.background_color.g = 255
  f.background_color.b = 255

  mouseTracking = 1 // for wheel events

  //_DEBUG_SEE_COMPONENTS_DESTRUCTION_INFO_LEVEL = 2

  Component toolbox {
    Translation pos_buttons(10,0)
    StandAlonePushButton del ("Delete last", 0, 0)
    StandAlonePushButton add ("Add Rectangle", 0, 0)
    f.height - 40 =:> pos_buttons.ty
    add.x + add.width + 10 =:> del.x
  }

  Int texts_list_width (250)

  ViewModelManager VM_manager ()

  // View for list of texts
  TextsListView texts_list_view (VM_manager)
  texts_list_width =: texts_list_view.width

  // View for list of rectangles
  RectanglesListView rectangles_list_view (VM_manager, f)
  texts_list_width + 4 =: rectangles_list_view.x
  f.width - rectangles_list_view.x =:> rectangles_list_view.width

  f.height =:> texts_list_view.height, rectangles_list_view.height


  toolbox.add.click -> VM_manager.new_rectangle

  toolbox.del.click -> VM_manager.delete_rectangle

}
