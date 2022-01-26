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

 _define_
 TextLineEdit (Process f, string default_text, double x, double y) {
  String _default_text (default_text)
  String validated_text ("")
  Translation pos (x, y)
  svg = loadFromXML ("drawing.svg")
  bkg << svg.layer1.text\-edit.bkg
  clear << svg.layer1.text\-edit.clear
  RectangleClip clip (1, 1, 180, 18)
  Translation back_move (0, 0)
  text_field << svg.layer1.text\-edit.text_field
  
  Switch sw_cursor (idle) {
    Component idle {
    _default_text =: text_field.text
    }
    Component edit {
      Bool is_wider (0)
      text_field.width > 175 => is_wider

      FSM text_pos_management {
        State idle {
          0 =: back_move.tx
        }
        State wider {
          -(text_field.width - 175) =:> back_move.tx
        }
        idle->wider (is_wider.true)
        wider->idle (is_wider.false)
      }
   
      /* Cursor */
      OutlineColor _ (0, 0, 0)
      OutlineWidth _ (1)
      Clock cl (500)
      FSM blink {
        State show {
          Line cursor (0, 3, 0, 17)
        }
        State hide
        show->hide (cl.tick)
        hide->show (cl.tick)
      }
    }
  }

  AssignmentSequence set_text (1) {
    text_field.text =: validated_text
  }
  TextField le (text_field, bkg)
  le.x_cursor + text_field.x =:> sw_cursor.edit.blink.show.cursor.x1, sw_cursor.edit.blink.show.cursor.x2

  Bool is_left (0)
  Bool is_right (0)
  Bool is_del (0)
  Bool is_suppr (0)
  Bool is_return (0)
  Bool is_shift_press (0)
  Bool is_shift_release (0)
  Bool del_or_replace (0)
  Switch sw_box_edit (idle) {
    Component idle {
      FillColor _ (160, 160, 160)
      Text t (2, 15, "")
      _default_text =:> text_field.text
      0 =: back_move.tx
      150 =: text_field.fill.r, text_field.fill.g, text_field.fill.b
    }
    Component edit {
      0 =: text_field.fill.r, text_field.fill.g, text_field.fill.b
      clear.shape.press->le.clear
      f.key\-pressed_text =>le.new_text
      f.key\-pressed == 16777223 => is_suppr
      f.key\-pressed == 16777219 => is_del
      f.key\-pressed == 16777236 => is_right
      f.key\-pressed == 16777234 => is_left
      f.key\-pressed == 16777220 => is_return
      f.key\-pressed == 16777248 => is_shift_press
      f.key\-released == 16777248 => is_shift_release

      is_suppr.true->le.suppr
      is_del.true->le.del
      is_right.true->le.move_right
      is_left.true->le.move_left

      is_suppr || is_del || f.key\-pressed < 16777216 || f.key\-pressed > 16908289 => del_or_replace

      Double x_select (0)

      /* Graphic shape for selection */
      FSM rec_selection {
        State no_rec
        State rec {
          FillOpacity _ (0.5)
          FillColor _ (50, 50, 200)
          Rectangle rec (0, 3, 0, 14, 0, 0)
          le.x_start + text_field.x =:> rec.x
          le.x_cursor + text_field.x - rec.x =:> rec.width
        }
        no_rec->rec (le.start_selection)
        rec->no_rec (le.end_selection)
      }
      
      FSM select_all_fsm {
        State idle
        State wait {
          Timer t (500)
          bkg.press->le.select_all
        }
        idle->wait (bkg.press)
        wait->idle (wait.t.end)
      }
      FSM select_fsm {
        State idle {
          is_right || is_left -> le.end_selection
        }
        State kbd_select
        State mouse_select {
          bkg.move->le.pointer_move
        }
        idle->mouse_select (bkg.press, le.start_selection)
        idle->kbd_select (is_shift_press.true, le.start_selection)
        kbd_select->idle (is_shift_release.true)
        mouse_select->idle (bkg.release)
      }
    }
  }

  /* Editing mode FSM */
  FSM fsm_box_edit {
    State idle
    State edit {
      FSM fsm_leave {
        State st_in
        State out {
          Spike end_edit
          f.press->end_edit
        }
        st_in->out (bkg.leave)
        out->st_in (bkg.enter)
      }
    }
    State leave
    idle->edit (bkg.press, le.clear)
    edit->idle (edit.fsm_leave.out.end_edit)
    edit->idle (is_return.true, set_text)
  }
  fsm_box_edit.state =:> sw_box_edit.state, sw_cursor.state
}
