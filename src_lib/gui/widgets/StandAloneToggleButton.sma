/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2022)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Poirier <mathieu.poirier@enac.fr>
 *
 */
use core
use base
use display
use gui


_define_
StandAloneToggleButton (string _label, double x_, double y_) {
  Translation t (x_, y_)

 /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  Spike toggle
  /*----- interface -----*/

  Int idle_color (#323232)
  Int pressed_color (#535353)
  FillColor fc (#323232)
  Rectangle r (0, 0, 100, 40, 5, 5)

  press aka r.press

  FSM fsm {
    State idle {
      idle_color =: fc.value
    }
    State pressed {
      pressed_color =: fc.value
    }
    idle->pressed (r.press, toggle)
    pressed->idle (r.press, toggle)
  }

  FillColor text_color (255, 255, 255)
  TextAnchor _ (DJN_MIDDLE_ANCHOR)
  FontFamily _ ("B612")
  FontSize _ (5, 13) // 5 to use the pixel unit. Allows to have the same rendering on MAC & Linux
  FontWeight _ (DJN_NORMAL)
  Text thisLabel (10, 10, _label)
  label aka thisLabel.text
  thisLabel.width + 20 =:> r.width
  thisLabel.height + 10 =:> r.height
  r.height/2.0 + (thisLabel.ascent - thisLabel.descent)/2.0 - 1 =:> thisLabel.y
  r.width / 2.0 =:> thisLabel.x

  width aka r.width
  height aka r.height
}