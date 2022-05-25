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
  OutlineColor _ (#535353)
  FillColor bg_color (White)
  Rectangle bg (0, 0, 100, 30, 3, 3)
  this.min_width = 100
  this.preferred_width = 100
  this.min_height = 30
  this.width =:> bg.width

  FillColor text_color (#535353)
  RectangleClip clip (1, 1, 180, 18)
  bg.width - 2 =:> clip.width
  bg.height =:> clip.height
  Translation back_move (0, 0)
  Text ui_text (10, 10, "")
  text aka ui_text.text
  bg.height/2.0 + (ui_text.ascent - ui_text.descent)/2.0 - 1 =:> ui_text.y
  Spike clear

  Bool is_wider (0)
  ui_text.width > (bg.width - 2) => is_wider

  FSM text_pos_management {
    State idle {
      0 =: back_move.tx
    }
    State wider {
      -(ui_text.width - bg.width - 2) =:> back_move.tx
    }
    idle->wider (is_wider.true)
    wider->idle (is_wider.false)
  }

  FSM fsm_cursor {
    State idle
    State edit {
      /* Cursor */
      OutlineColor _ (0, 0, 0)
      OutlineWidth _ (1)
      Clock cl (500)
      FSM blink {
        State show {
          Line cursor (2, 3, 2, 17)
          ui_text.y - ui_text.ascent - 1 =:> cursor.y1
          ui_text.height + cursor.y1 + 2 =:> cursor.y2
        }
        State hide
        show->hide (cl.tick)
        hide->show (cl.tick)
      }
      Spike quit
      FSM in_out {
        State in
        State out {
          GenericMouse.left.press->quit
        }
        in->out (bg.leave)
        out->in (bg.enter)
      }
    }
    idle->edit (bg.press)
    edit->idle (edit.quit)
  }
  TextField le (ui_text, bg)
  le.x_cursor + ui_text.x =:> fsm_cursor.edit.blink.show.cursor.x1, fsm_cursor.edit.blink.show.cursor.x2

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
      0 =: back_move.tx
      #969696 =: text_color.value
    }
    Component edit {
      #535353 =: text_color.value
      GenericKeyboard.key\-pressed_text =>le.new_text
      GenericKeyboard.key\-pressed == 16777223 => is_suppr
      GenericKeyboard.key\-pressed == 16777219 => is_del
      GenericKeyboard.key\-pressed == 16777236 => is_right
      GenericKeyboard.key\-pressed == 16777234 => is_left
      GenericKeyboard.key\-pressed == 16777220 => is_return
      GenericKeyboard.key\-pressed == 16777248 => is_shift_press
      GenericKeyboard.key\-released == 16777248 => is_shift_release

      is_suppr.true->le.suppr
      is_del.true->le.del
      is_right.true->le.move_right
      is_left.true->le.move_left

      is_suppr || is_del || GenericKeyboard.key\-pressed < 16777216 || GenericKeyboard.key\-pressed > 16908289 => del_or_replace

      Double x_select (0)

      /* Graphic shape for selection */
      FSM rec_selection {
        State no_rec
        State rec {
          FillOpacity _ (0.5)
          NoOutline _
          FillColor _ (#0157FF)
          Rectangle rec (0, 3, 0, 14, 0, 0)
          ui_text.y - ui_text.ascent - 1 =:> rec.y
          ui_text.height + 4 =:> rec.height
          le.x_start + ui_text.x =:> rec.x
          le.x_cursor + ui_text.x - rec.x =:> rec.width
        }
        no_rec->rec (le.start_selection)
        rec->no_rec (le.end_selection)
      }
      
      FSM select_all_fsm {
        State idle
        State wait {
          Timer t (500)
          bg.press->le.select_all
        }
        idle->wait (bg.press)
        wait->idle (wait.t.end)
      }
      FSM select_fsm {
        State idle {
          is_right || is_left -> le.end_selection
        }
        State kbd_select
        State mouse_select {
          bg.move->le.pointer_move
        }
        idle->mouse_select (bg.press, le.start_selection)
        idle->kbd_select (is_shift_press.true, le.start_selection)
        kbd_select->idle (is_shift_release.true)
        mouse_select->idle (bg.release)
      }
    }
  }
  fsm_cursor.state =:> sw_box_edit.state
}