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

_native_code_
%{
#include <assert.h>
%}

_define_
TextsListView (Process _view_model_manager)
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

  Text label (10, 15, "0 rectangle in the model")
  view_model_manager.model_manager.models_list.size + 
  (view_model_manager.model_manager.models_list.size <= 1
    ? " rectangle"
    : " rectangles")
    + " in the model" => label.text

  List views_list

  Int delta_y (20)

  // When a ViewModel is added to the list of ViewModels
  view_model_manager.view_models_list.$added -> na_view_models_list_added:(this) {
    //print ("(TextsList)View view_models_list added (avant): " + this.views_list.size + " Vs")
    src = &this.view_model_manager.view_models_list.$added
    view_model = getRef (&src)
    assert (&view_model)
    
    // create a new view
    Process view = TextView (this.views_list, "", view_model, $this.delta_y)

    // set next position
    this.delta_y += 20
    //print ("(TextsList)View view_models_list added (apres): " + this.views_list.size + " Vs")
  }

  // When a ViewModel is removed from the list of ViewModels
  view_model_manager.view_models_list.$removed -> na_view_models_list_removed:(this) {
    //print ("(TextsList)View view_models_list removed (avant): " + this.views_list.size + " Vs")
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

    //print ("(TextsList)View view_models_list removed (apres): " + this.views_list.size + " Vs")
  }

  // then update the y layout
  na_view_models_list_removed -> (this) {
      // Reset y of remaining views
      this.delta_y = 20
      for view : this.views_list {
        view.y = this.delta_y
        this.delta_y += 20
      }
  }

}