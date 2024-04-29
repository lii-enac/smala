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

_define_
RectView (Process _view_model) {
  vm aka _view_model

  //TextPrinter tp

  OutlineColor outline (#888888)
  OutlineWidth w (10)

  FillColor fill (#FF0000)

  Rectangle r (0, 0, 0, 0)

  // update the view whenever the view model changes
  vm.x =:> r.x
  vm.y =:> r.y
  vm.width =:> r.width
  vm.height =:> r.height


  Double off_x (0)
  Double off_y (0)

  FSM fsm {
    State st_idle {
      #FF0000 =: fill.value
    }

    State st_press {
      #EE0000 =: fill.value

      r.press.x - r.x =: off_x
      r.press.y - r.y =: off_y
      //"offset: " + off_x + " - " + off_y =: tp.input
    }

    State st_dragging {
      #DD0000 =: fill.value

      r.move.x - off_x =:> vm.x
      r.move.y - off_y =:> vm.y
      //"move: " + r.move.x + " - " + r.move.y =: tp.input
    }

    st_idle -> st_press (r.press)
    st_press -> st_idle (r.release)
    st_press -> st_dragging (r.move.x)
    st_dragging -> st_idle (r.release)
  }
  

}