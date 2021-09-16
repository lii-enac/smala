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
import gui.widgets.IWidget

_define_
RadioButton (Process container, string _label, double x_, double y_) inherits IWidget (container) {
  Translation t (x_, y_)

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  Spike selected
  Spike unselected
  Spike unselect
  /*----- interface -----*/

  Int check_color (#535353)
  Int idle_color (#ffffff)

  OutlineWidth _ (2)
  OutlineColor oc (#535353)
  FillColor fc (#ffffff)
  Circle r (9, 9, 9)

  Spike press
  r.press->press

  FSM fsm {
    State idle 
    State selected {
      FillColor fc2 (#535353)
      NoOutline _
      Circle r (9, 9, 6)
    }
    idle->selected (press, selected)
    selected->idle (press, unselected)
    selected->idle (unselect)
  }

  FillColor text_color (#323232)
  Text thisLabel (23, 16, _label)
  thisLabel.press->press
  label aka thisLabel.text
  thisLabel.width + 33 =:> this.min_width
  30 =:> this.min_height
}
