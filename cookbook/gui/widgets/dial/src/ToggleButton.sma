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

_native_code_
%{
extern char* buildPath (const char *file);
%}

_define_
ToggleButton (Process frame, string l, double _x, double _y) {
  Translation t (_x, _y)

  /*----- interface -----*/
  Spike on
  Spike off
  x aka t.tx
  y aka t.ty
  Double getwidth (0)
  Double getheight (0)
  String label (l)
 /*--- end interface ---*/

  gbutton = loadFromXML (buildPath("img/button.svg"))
  FSM fsm {
    State idle {
      r = addChild (gbutton.idle)
    }
    State pressed {
      r = addChild (gbutton.pressed)
    }
    idle->pressed (idle.r.press, on)
    pressed->idle (pressed.r.press, off)
  }
  glabel = addChild (gbutton.label)
  label =:> glabel.text
  glabel.width + 30 =:> fsm.idle.r.width, fsm.pressed.r.width, getwidth
  fsm.idle.r.height =:> getheight
  fsm.idle.r.width / 2 =:> glabel.x
}
