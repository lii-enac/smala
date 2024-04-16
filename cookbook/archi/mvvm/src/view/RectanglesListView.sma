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
use gui

import RectView

_define_
RectanglesListView (Process _view_model_manager)
{ 
  view_model_manager aka _view_model_manager

  Translation pos (0, 0)
  x aka pos.tx
  y aka pos.ty

  Component bg {
    OutlineColor out_c (#000000)
    OutlineWidth out_w (2)
    NoFill _

    Rectangle r (0, 0, 100, 100, 10, 10)
  }
  width aka bg.r.width
  height aka bg.r.height

  List views_list

  view_model_manager.view_models_list.$added -> na_view_models_list_added:(this) {
    view_model = getRef (&this.view_model_manager.view_models_list.$added)
    //view_model = getRef (&src)
    if (&view_model != null) {
      print ("(RectanglesList)View view_models_list added (avant): " + this.views_list.size + " Vs")
      Process view = RectView (this.views_list, "", view_model)
      print ("(RectanglesList)View view_models_list added (apres): " + this.views_list.size + " Vs")
    }
  }

  view_model_manager.view_models_list.$removed -> na_view_models_list_removed:(this) {
    view_model = getRef (&this.view_model_manager.view_models_list.$removed)
    //view_model = getRef (&src)
    if (&view_model != null) {
      print ("(RectanglesList)View view_models_list removed (avant): " + this.views_list.size + " Vs")
      for view : this.views_list {
        if (&view.vm == &view_model) {
          delete view
          break
        }
      }
      print ("(RectanglesList)View view_models_list removed (apres): " + this.views_list.size + " Vs")
    }
  }
}
