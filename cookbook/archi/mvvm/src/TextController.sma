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
TextController(Process model, Process _view, Process _view_model)
{
  view aka _view
  view_model aka _view_model

  Component control {

    // update the view_model whenever the model changes (subject/observer pattern)
    model.{x,y,width,height} =:> view_model.{x,y,width,height}

    // since the view_model acts as a proxy, we must update the model whenever view_model changes
    // this introduces a cycle that we must allow
    _AUTHORIZE_CYCLE = 1
    // FIXME test if value is the same and do not warn! (would it work? not sure...)
    // FIXME _AUTHORIZE_CYCLE should be limited to this component
    // __AUTHORIZE_IDEMPOTENT_CYCLE = 1
    view_model.{x,y,width,height} =:> model.{x,y,width,height}

    // setup a bidirectional connector between the view and the the view_model
    // in this example, it's not strictly necessary, since the view is modified only through interaction
    // still, this introduces a cycle that we must allow
    _AUTHORIZE_CYCLE = 1
    // FIXME test if value is the same and do not warn! (would it work? not sure...)
    // __AUTHORIZE_IDEMPOTENT_CYCLE = 1
    view_model.{x,y,width,height} =:> view.{x,y,width,height}
          view.{x,y,width,height} =:> view_model.{x,y,width,height}

    // update the view_model from interactions on the view
         view.t_x.wheel.dy +=> view_model.x
         view.t_y.wheel.dy +=> view_model.y
     view.t_width.wheel.dy +=> view_model.width
    view.t_height.wheel.dy +=> view_model.height
  }

  Spike about_to_delete
  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this.view_model
    delete this
  }
}