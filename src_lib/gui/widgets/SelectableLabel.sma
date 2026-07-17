/*
*  Smala Library
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui

import gui.widgets.IWidget

_define_
SelectableLabel (string _label) inherits IWidget () {
  mouseTracking = 1

  Spike leave
  Int text_color (#FFFFFF)
  Int text_selected_color (#FFFFFF)
  Int selection_color (#429FE0)

  Translation offset (0, 0)
  TextField field (0, 0, 1, 1, _label, 1)
  field.read_only = 1

  text aka field.content.text
  read_only aka field.read_only
  selectable aka field.selectable
  press aka field.press
  text_color =:> field.text_color
  text_selected_color =:> field.text_selected_color
  selection_color =:> field.selection_color

  this.height/2 - field.content_height/2 =:> offset.ty
  // In this case, we want the text_field to be sized according to its content
  field.content_width =:> field.width, this.width, this.min_width
  field.content_height =:> field.height, this.height, this.min_height

  // Generate leave when the user clicks outside the field, so only one label owns keyboard shortcuts.
  FSM in_out {
    State out {
      GenericMouse.left.press -> leave
    }
    State in
    out->in (field.enter)
    in->out (field.leave)
  }

  FSM focus {
    State idle
    State active {
      // Forward keyboard shortcuts such as Ctrl+C while this label owns focus.
      GenericKeyboard.key\-pressed => field.key_pressed
      GenericKeyboard.key\-released => field.key_released
    }
    idle->active (field.press)
    active->idle (leave)
  }

}
