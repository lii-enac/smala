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
Button (Process frame, string l, double _x, double _y) {
  Translation t (_x, _y)

  /*----- interface -----*/
  Spike click
  x aka t.tx
  y aka t.ty
  Double getwidth (0)
  Double getheight (0)
  TextProperty label (l)
  /*--- end interface ---*/

  /*----- graphics and behavior-----*/
  gbutton = load_from_XML ("img/button.svg")

  FSM fsm {
    State idle {
      r << gbutton.idle
    }
    State pressed {
      r << gbutton.pressed
    }
    idle->pressed (idle.r.press)
    pressed->idle (pressed.r.release, click)
    pressed->idle (frame.release)
  }
  glabel << gbutton.label
  label =:> glabel.text
  glabel.width + 30 =:> fsm.idle.r.width, fsm.pressed.r.width, getwidth
  fsm.idle.r.height =:> getheight
  fsm.idle.r.width / 2 =:> glabel.x
}
