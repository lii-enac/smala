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
CheckBox (string _label) inherits IWidget () {

  /*----- interface -----*/
  Spike selected
  Spike unselected
  Spike unselect
  /*----- interface -----*/

  Int check_color (#535353)
  Int idle_color (#ffffff)

  Translation offset (0, 0)
  
  FillColor text_color (#323232)
  Text thisLabel (23, 14, _label)
  OutlineWidth ow (2)
  OutlineColor oc (#535353)
  FillColor fc (#ffffff)
  
  Rectangle r (0, 0, 16, 16, 2, 2)
  this.height/2 - r.height/2 =:> offset.ty

  Spike press
  r.press -> press

  FSM fsm {
    State st_idle {
      idle_color =: fc.value
      //check_color =: oc.value
      2 =: ow.width
    }
    State st_selected {
      //idle_color =: oc.value
      check_color =: fc.value
      1 =: ow.width
      OutlineColor oc (#ffffff)
      OutlineJoinStyle _ (1)
      OutlineCapStyle _ (1)
      OutlineWidth _ (2)
      Polyline p {
        Point _ (4, 8)
        Point _ (8, 13)
        Point _ (13, 4)
      }
    }
    st_idle->st_selected (press, selected)
    st_selected->st_idle (press, unselected)
    st_selected->st_idle (unselect)
  }

  

  thisLabel.press->press
  label aka thisLabel.text
  thisLabel.width + 23 =:> this.min_width
  this.min_width = thisLabel.width + 23
  this.min_height = 16
}
