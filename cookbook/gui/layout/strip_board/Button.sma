/*
 *  StripBoard app
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
 
use core
use exec_env
use base
use display
use gui

_define_
Button (Process frame, string label, double x_, double y_) {
  Translation t (x_, y_)

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  Spike click
  /*----- interface -----*/

  Int idle_color (#41444C)
  Int pressed_color (#828799)
  FillColor fc (#41444C)
  Rectangle r (0, 0, 50, 30, 5, 5)

  FSM fsm {
    State idle {
      idle_color =: fc.value
    }
    State pressed {
      pressed_color =: fc.value
    }
    idle->pressed (r.press)
    pressed->idle (r.release, click)
    pressed->idle (frame.release)
  }

  FillColor w (255, 255, 255)
  TextAnchor _ (DJN_MIDDLE_ANCHOR)
  Text thisLabel (10, 10, label)

  r.height / 2 + 5 =: thisLabel.y
  thisLabel.width > 50 ? thisLabel.width + 20 : 50 =: r.width
  r.width / 2 =: thisLabel.x

}
