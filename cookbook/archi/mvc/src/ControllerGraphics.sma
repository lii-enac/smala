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
use gui

_define_
ControllerGraphics(Process _model, Process _view)
{
  model aka _model
  view  aka _view
  
  Spike about_to_delete

  Component control {
    // update view from model (subject/observer pattern)
    model.{x,y,width,height} =:> view.r.{x,y,width,height}

    // update model from interactions on the view
    FSM fsm {
      State idle
      State drag {
        Int off_x(0)
        Int off_y(0)
        view.r.press.x - model.x =:  off_x
        view.r.press.y - model.y =:  off_y
        view.r.move.x  - off_x   =:> model.x
        view.r.move.y  - off_y   =:> model.y
      }
      idle->drag(view.r.press)
      drag->idle(view.r.release)
    }

    FSM selection {
      State unselected {
        #FF0000 =: view.fill.value
      }
      State selected {
        #00FF00 =: view.fill.value
        GenericKeyboard.key\-pressed == DJN_Key_Delete->about_to_delete
      }
      unselected->selected(view.r.press)
      selected->unselected(view.r.press)
    }
  }


  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this
  }
}