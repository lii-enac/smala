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

_define_
TextController(Process model, Process _view)
{
  view aka _view
  Component control {
    // update the view whenever the model changes (subject/observer pattern)
    model.{x,y,width,height} =:> view.{x,y,width,height}

    // update model from interactions on the view
         view.t_x.wheel.dy +=> model.x // FIXME/TODO
         view.t_y.wheel.dy +=> model.y
     view.t_width.wheel.dy +=> model.width
    view.t_height.wheel.dy +=> model.height
  }

  Spike about_to_delete
  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this
  }
}