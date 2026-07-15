/*
*  Smala cookbook mvvm_v2
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023-2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base
use gui

import RectView

_native_code_
%{
#include <assert.h>
%}

_define_
RectanglesListView (Process _view_model_manager, Process _frame)
{ 
  view_model_manager aka _view_model_manager
  frame aka _frame

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

  // When a ViewModel is added to the list of ViewModels
  view_model_manager.view_models_list.$added -> na_view_models_list_added:(this) {
    //print ("(RectanglesList)View view_models_list added (avant): " + this.views_list.size + " Vs")
    src = &this.view_model_manager.view_models_list.$added
    view_model = getRef (&src)
    assert (&view_model != null)

    // create a new View
    Process view = RectView (this.views_list, "", view_model, this.frame)
    //print ("(RectanglesList)View view_models_list added (apres): " + this.views_list.size + " Vs")
  }

  // When a ViewModel is removed from the list of ViewModels
  view_model_manager.view_models_list.$removed -> na_view_models_list_removed:(this) {
    //print ("(RectanglesList)View view_models_list removed (avant): " + this.views_list.size + " Vs")
    src = &this.view_model_manager.view_models_list.$removed
    view_model = getRef (&src)
    assert (&view_model)

    // find the view corresponding to this view_model
    Process view = null
    for v : this.views_list {
      if (&v.vm == &view_model) {
        view = &v
        break
      }
    }
    assert(&view)

    // delete it
    delete view
    //print ("(RectanglesList)View view_models_list removed (apres): " + this.views_list.size + " Vs")
  }
}
