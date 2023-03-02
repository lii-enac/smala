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
  Spike click   // FIXME? should be past simple: 'clicked', or better: 'triggered' ("less" linked to the particular interaction that triggered it)
  Spike release // FIXME? should be past simple: 'released'
  Spike select  // _no_ FIXME: this is one is an action(?)
  // Spike trigger // action: should animate a triggering
  // Spike silent_trigger // action: should trigger with no animation

  /*----- interface -----*/

  Int disabled_color (#959595)
  Int idle_color (#323232)
  Int pressed_color (#959595)
  Int text_color (#ffffff)
  OutlineColor _ (#535353)
  FillColor fc (#535353)
  Rectangle r (0, 0, 100, 40, 3, 3)

  press aka r.press
  Bool ret_key_pressed (0)
  Bool ret_key_released (0)
  GenericKeyboard.key\-pressed == DJN_Key_Return => ret_key_pressed
  GenericKeyboard.key\-released == DJN_Key_Return => ret_key_released
  FSM fsm {
    State idle {
      idle_color =: fc.value
    }
    State disabled {
      disabled_color =: fc.value
    }
    State selected {
      OutlineColor _ (White)
      double[] pattern = [2.0, 2.0]
      DashArray dash (pattern)
      Rectangle r_dash (3, 3, 94, 34, 3, 3)
      r.width - 6 =:> r_dash.width
      r.height - 6 =:> r_dash.height
    }
    State pressed {
      pressed_color =: fc.value
      r.release->release
    }
    State out {
      idle_color =: fc.value
    }
    disabled -> idle     (this.enable,  this.enabled)
        idle -> disabled (this.disable, this.disabled)
    selected -> disabled (this.disable, this.disabled)
     pressed -> disabled (this.disable, this.disabled)
         out -> disabled (this.disable, this.disabled)

    idle->pressed (r.press)
    idle->selected (select)
    selected->pressed (r.press)
    selected->pressed (ret_key_pressed.true)
    pressed->idle (r.release, click)
    pressed->idle (ret_key_released, click)
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
  this.preferred_width = thisLabel.width + 20
  thisLabel.height + 10 =:> this.min_height
  this.min_height = thisLabel.height + 10
  this.preferred_height = thisLabel.height + 10
  this.width =:> r.width
  this.height =:> r.height
  r.height/2.0 + (thisLabel.ascent - thisLabel.descent)/2.0 - 1 =:> thisLabel.y
  r.width / 2.0 =:> thisLabel.x
}
