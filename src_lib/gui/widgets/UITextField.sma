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
  mouseTracking = 1   // Allow to know if the mouse/cursor is hover our text field or not
  
  Spike validate  // Key "Return" allows to validate the text
  Spike clear     // Clear our text field
  Spike next      // Key "Tab" allows to give focus to another widget
  Spike activate  // Allow to force to give focus to our text field
  Spike leave     // When user press outside our text field

  String text ("")      // current value of our text field
  String init_text ("") // initial value of our text field


  OutlineColor bg_ol_color (#535353)
  FillColor bg_color (White)  

  Int default_width (190)
  Int default_height (21)
  Int horizontal_padding (5)

  this.width = default_width
  this.height = default_height

  Rectangle bkg (0, 0, 190, 21, 3, 3)
  Translation content_offset (5, 2)
  FillColor _ (Black)
  
  TextField field (0, 0, 180, 18, "", 1)
  field.validate -> validate  // propagate validate from inner text field
  clear -> field.clear        // our signal clear the inner text field

  this.preferred_height = default_height
  this.min_height = default_height
  this.min_width = 100
  this.width =:> bkg.width
  this.height =:> bkg.height
  this.width - 2*horizontal_padding =:> field.width
  this.height - content_offset.ty =:> field.height
  this.height > field.cursor_height ? (this.height - field.cursor_height) / 2 : 0 =:> content_offset.ty

  Int unedit_text_color (0) //(#909090)
  Int edit_text_color (0)
  Int disabled_color (#959595)

  text_color aka field.text_color // IntProperty
  text_selected_color aka field.text_selected_color // IntProperty
  selection_color aka field.selection_color // IntProperty

  // Manage when the mouse/cursor is hover our text field or not
  FSM in_out {
    State out {
      GenericMouse.left.press -> leave
    }
    State in
    out->in (field.enter)
    in->out (field.leave)
  }

  AssignmentSequence set_text (1) {
    field.content.text =: text
  }
  field.clear -> set_text

  FSM edit_fsm {
    State no_edit {
      unedit_text_color =: text_color
      |-> field.disable_edit
    }
    State disabled {
      disabled_color =: text_color
    }
    State edit {
      |-> field.enable_edit
      edit_text_color =: text_color
      /* Cursor */
      OutlineColor _ (Black)
      OutlineWidth _ (1)
      Line cursor (0, 0, 0, 15)
      field.cursor_end_x =:> cursor.x1, cursor.x2
      field.cursor_height =:> cursor.y2

      validate -> leave   // Key "Return" --> validate -> leave --> change state to no_edit
      next -> validate

      GenericKeyboard.key\-pressed => field.key_pressed
      GenericKeyboard.key\-released => field.key_released
      GenericKeyboard.key\-pressed_text => field.string_input
      GenericKeyboard.key\-pressed == DJN_Key_Tab -> next
    }
    no_edit->edit (field.press)
    no_edit->edit (bkg.press)
    no_edit->edit (activate)
    edit->no_edit (leave, set_text)   // Update current value of our text
    edit->no_edit (next, set_text)    // Update current value of our text
    
    disabled -> no_edit (this.enable, this.enabled)
    { no_edit, edit } -> disabled (this.disable, this.disabled)
  }

  // Init the inner text field
  init_text =:> field.content.text, text
  
}
