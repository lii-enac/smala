use core
use base
use gui

_define_
MyBox (Process root, int _x, int _y) {
  String title ("Title")
  FillColor color (#101010)
  OutlineColor stroke (#707070)
  OutlineWidth _ (3)

  Rectangle r (_x, _y, 100, 100, 5, 5)
  x aka r.x
  y aka r.y
  width aka r.width
  height aka r.height
  TextAnchor _ (1)
  

  FSM fsm_edit {
    State idle {
      FillColor _(White)
      Text idle_title (50, 50, "Title")
      title =: idle_title.text
      r.x + r.width/2 =:> idle_title.x
      r.y + r.height/2 + idle_title.ascent/2 =:> idle_title.y
      idle_title.width + 20 > 100 ? idle_title.width + 20 : 100 =:> r.width
    }
    State edit {
      RectangleClip clip (0, 0, 0, 0)
      r.{y,height} =: clip.{y,height}
      r.x+3 =: clip.x
      r.width - 6 =: clip.width

      Translation pos (0, 0)
      r.y + r.height/2 - idle.idle_title.ascent/2 =: pos.ty
      
      r.x + 3 =: pos.tx
      TextField field (0, 0, 1000, 18, "", 1)
      r.width - 6 =: field.width
      title =: field.content.text
      #FFFFFF =: field.text_color
      AssignmentSequence set_text (1) {
        field.content.text =: title
      }
      OutlineColor _ (White)
      OutlineWidth _ (1)
      Line cursor (0, 0, 0, 15)
      field.cursor_end_x =:> cursor.x1, cursor.x2
      field.cursor_height =:> cursor.y2
      Spike escape
      GenericKeyboard.key\-pressed => field.key_pressed
      GenericKeyboard.key\-released => field.key_released
      GenericKeyboard.key\-pressed_text => field.string_input
      GenericKeyboard.key\-pressed == DJN_Key_Return -> set_text
      TextPrinter tp
      GenericKeyboard.key\-pressed == 16777216 -> escape
    }
    idle->edit (idle.idle_title.press)
    edit->idle (edit.set_text)
    edit->idle (edit.escape)
  }
  

  Switch sw_mode (idle) {
    Component idle {
      FSM dnd {
        State idle
        State moving {
          Double init_x (0)
          Double init_y (0)
          Double off_x (0)
          Double off_y (0)
          r.x =: init_x
          r.y =: init_y
          r.press.x =: off_x
          r.press.y =: off_y
          r.move.x - off_x + init_x => r.x
          r.move.y - off_y + init_y => r.y
        }
        idle->moving (r.press)
        moving->idle (r.release)
      }
    }
    Component search_target{
      r.press -> {r =: root.cur_box}
    }
    root.fsm_mode.state =:> sw_mode.state
  }
  


  Spike add_arrow
  add_arrow->{r =: root.cur_box}

  NoFill _
  OutlineOpacity op (0)
  OutlineColor _(#AFAFAF)
  OutlineWidth _(4)
  Rectangle border (0, 0, 0, 0, 5, 5)
  r.x  =:> border.x
  r.y  =:> border.y
  r.width  =:> border.width
  r.height =:> border.height
  FSM border_fsm {
    State no_show {
      0 =: op.a
    }
    State show {
      1 =: op.a
      border.press -> add_arrow
    }
    no_show->show (border.enter)
    show->no_show (border.leave)
  }
}