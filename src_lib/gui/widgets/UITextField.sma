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
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
use core
use base
use display
use gui

import gui.widgets.IWidget

_define_
UITextField () inherits IWidget () {
  mouseTracking = 1
  Spike validate
  Spike clear
  Spike next
  Spike activate
  String text ("")
  String init_text ("")


  OutlineColor _ (#535353)
  FillColor bg_color (White)  

  Rectangle bkg (0, 0, 190, 21, 3, 3)
  Translation _ (5, 2)
  FillColor _ (Black)
  TextField field (0, 0, 180, 18, "", 1)
  this.preferred_height = 18
  this.width =:> bkg.width
  this.width - 10 =:> field.width

  Int unedit_text_color (#909090)
  Int edit_text_color (0)
  text_color aka field.text_color // IntProperty
  text_selected_color aka field.text_selected_color // IntProperty
  selection_color aka field.selection_color // IntProperty

  Spike leave
  FSM in_out {
    State out {
      GenericMouse.left.press->leave
    }
    State in
    out->in (field.enter)
    in->out (field.leave)
  }

  AssignmentSequence set_text (1) {
    field.content.text =: text
  }
  validate -> set_text

  FSM edit_fsm {
    State no_edit {
      unedit_text_color =: text_color
    }
    State edit {
      edit_text_color =: text_color
      /* Cursor */
      OutlineColor _ (Black)
      OutlineWidth _ (1)
      Line cursor (0, 0, 0, 15)
      field.cursor_end_x =:> cursor.x1, cursor.x2
      field.cursor_height =:> cursor.y2
      validate->leave

      GenericKeyboard.key\-pressed => field.key_pressed
      GenericKeyboard.key\-released => field.key_released
      GenericKeyboard.key\-pressed_text => field.string_input
      GenericKeyboard.key\-pressed == DJN_Key_Tab -> next, set_text
    }
    no_edit->edit (field.press)
    no_edit->edit (bkg.press)
    no_edit->edit (activate)
    edit->no_edit (leave)
    edit->no_edit (next)
  }

  clear->field.clear
  init_text =:> field.content.text
  field.validate->validate
}