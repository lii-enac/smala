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
use gui

_define_
Button (Component frame, string label, double x_, double y_) {
  Component click
  Translation t (x_, y_)

  x aka t.tx
  y aka t.ty

  FSM fsm {
    State idle {
      FillColor fc (50, 50, 50)
      Rectangle r (0, 0, 100, 70, 10, 10)
    }
    State pressed {
      FillColor fc (150, 50, 50)
      Rectangle r (0, 0, 100, 70, 10, 10)
    }
    idle->pressed (idle.r.press)
    pressed->idle (pressed.r.release, click)
    pressed->idle (frame.release)
  }
  FillColor w (255, 255, 255)
  Text thisLabel (10, 10, label)
  fsm.idle.r.height / 2 =: thisLabel.y
  thisLabel.width + 20 => fsm.idle.r.width
  thisLabel.width + 20 => fsm.pressed.r.width
}
