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
use gui

_define_
View1(Process model) {
  Rectangle r (0,0,0,0)
  model.{x,y,width,height}=:>r.{x,y,width,height}
  FSM fsm {
    State idle
    State drag {
      Int off_x(0)
      Int off_y(0)
      r.press.x - model.x  =: off_x
      r.press.y - model.y =: off_y
      r.move.x - off_x =:> model.x
      r.move.y - off_y =:> model.y
    }
    idle->drag(r.press)
    drag->idle(r.release)
  }
}