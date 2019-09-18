/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
use core
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

  FillColor fc (50, 50, 50)
  Rectangle r (0, 0, 100, 70, 10, 10)

  FSM fsm {
    State idle {
      50 =: fc.r
    }
    State pressed {
      150 =: fc.r
    }
    idle->pressed (r.press)
    pressed->idle (r.release, click)
    pressed->idle (frame.release)
  }

  FillColor w (255, 255, 255)
  Text thisLabel (10, 10, label)

  r.height / 2 =: thisLabel.y
  thisLabel.width + 20 =:> r.width
  thisLabel.width + 20 =:> r.width
}
