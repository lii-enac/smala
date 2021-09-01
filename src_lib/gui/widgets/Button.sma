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

//import gui.shape.text

_define_
Button (Process frame, string _label, double x_, double y_) {
  Translation t (x_, y_)

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  Spike click
  Spike release
  /*----- interface -----*/

  Int idle_color (#323232)
  Int pressed_color (#535353)
  FillColor fc (#323232)
  Rectangle r (0, 0, 100, 40, 5, 5)

  press aka r.press

  Double height (15)
  Double width (100)
  ClampMin clamp_width (0, 0)
  ClampMin clamp_height (20, 20) 
  min_width aka clamp_width.min
  min_height aka clamp_height.min
  Double max_width (0)
  Double max_height (0)
  width =:> clamp_width.input
  height =:> clamp_height.input

  FSM fsm {
    State idle {
      idle_color =: fc.value
    }
    State pressed {
      pressed_color =: fc.value
      r.release->release
    }
    State out {
      idle_color =: fc.value
    }
    idle->pressed (r.press)
    pressed->idle (r.release, click)
    pressed->out (r.leave)
    out->pressed (r.enter)
    out->idle (r.release)
  }

  FillColor w (255, 255, 255)
  TextAnchor _ (DJN_MIDDLE_ANCHOR)
  Text thisLabel (10, 10, _label)
  label aka thisLabel.text
  thisLabel.width + 20 =:> clamp_width.min
  clamp_width.result =:> r.width
  clamp_height.result =:> r.height
  r.height/2.0 + (thisLabel.ascent - thisLabel.descent)/2.0 - 1 =:> thisLabel.y
  r.width / 2.0 =:> thisLabel.x
}
