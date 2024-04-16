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

import TextView

_define_
TextsListView (Process _view_model_manager)
{
  view_model_manager aka _view_model_manager

  Component bg {
    OutlineColor out_c (#000000)
    OutlineWidth out_w (1)
    NoFill _

    Rectangle r (0, 0, 100, 100)
  }
  x aka bg.r.x
  y aka bg.r.y
  width aka bg.r.width
  height aka bg.r.height

  Text label (10, 15, "0 rectangles in the model")
  view_model_manager.model_manager.models_list.size + " rectangles in the model" => label.text

  List views_list

  Int delta_y (15)

  // toolbox.add.click -> (root) {

  //   Process tv = TextView (root.views, "", $root.text_y)

  view_model_manager.view_models_list.$added -> na_view_models_list_added:(this) {
    view_model = getRef (&this.view_model_manager.view_models_list.$added)
    //view_model = getRef (&src)
    if (&view_model != null) {
      print ("(TextsList)View view_models_list (avant): " + this.views_list.size + " Vs")
      Process view = TextView (this.views_list, "", view_model, $this.delta_y)
      print ("(TextsList)View view_models_list (apres): " + this.views_list.size + " Vs")

      this.delta_y += 15
    }
  }

  view_model_manager.view_models_list.$removed -> na_view_models_list_removed:(this) {
    view_model = getRef (&this.view_model_manager.view_models_list.$removed)
    //view_model = getRef (&src)
    if (&view_model != null) {
      print ("VM models_list_removed (avant): " + this.views_list.size + " Vs")
      for view : this.views_list {
        if (&view.vm == &view_model) {
          delete view
          break
        }
      }
      print ("VM models_list_removed (apres): " + this.views_list.size + " Vs")
    }
  }

}