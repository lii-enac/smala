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

import gui.widgets.IWidget

_define_
PushButton (string _label) inherits IWidget () {
  /*----- interface -----*/
  Spike click
  Spike release
  /*----- interface -----*/

  Int idle_color (#323232)
  Int pressed_color (#535353)
  Int text_color (#ffffff)
  FillColor fc (#323232)
  Rectangle r (0, 0, 100, 40, 5, 5)

  press aka r.press

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

  FillColor tc (255, 255, 255)
  text_color =: tc.value
  TextAnchor _ (DJN_MIDDLE_ANCHOR)
  Text thisLabel (10, 10, _label)
  label aka thisLabel.text
  thisLabel.width + 20 =:> this.min_width
  this.min_width = thisLabel.width + 20
  thisLabel.height + 10 =:> this.min_height
  this.min_height = thisLabel.height + 10
  this.width =:> r.width
  this.height =:> r.height
  r.height/2.0 + (thisLabel.ascent - thisLabel.descent)/2.0 - 1 =:> thisLabel.y
  r.width / 2.0 =:> thisLabel.x
}
