/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2023)
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


_main_
Component root {
  Frame f ("Cursor", 0, 0, 500, 500)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1
  
  Cursor c("img/moon.png", 0, 0)
  String path("img/moon.png")
  
  f.background_color.r = 245
  f.background_color.g = 245
  f.background_color.b = 245

  FillColor _(200, 30, 30)
  Rectangle r (200, 200, 100, 100, 2, 2)
  FSM dnd {
    State idle
    State drag {
      Double off_x (0)
      Double off_y (0)
      r.press.x - r.x  =: off_x
      r.press.y - r.y =: off_y
      r.move.x - off_x =:> r.x
      r.move.y - off_y =:> r.y
    }
    idle->drag (r.press)
    drag->idle (r.release)
  }
  ClampMin rounded (0, 0)
  rounded.result => r.rx, r.ry
  r.wheel.dy -> {r.wheel.dy + r.rx =: rounded.input}

  NoFill _
  PickFill _
  Rectangle pick(0, 0, 15, 15)
  r.x + r.width - 15 =:> pick.x
  r.y + r.height - 15 =:> pick.y



  FSM pick_cursor_shape {
    State normal {
      path =: c.path
    }
    State resize {
      DJNN_SIZE_FDIAG =: c.cursor_shape
    }
    normal->resize(pick.enter)
    resize->normal(pick.leave)
  }

  FSM rect_cursor_shape {
    State normal {
      path =: c.path
    }
    State hover {
      DJNN_OPEN_HAND =: c.cursor_shape
    }
    State drag {
      DJNN_CLOSED_HAND =: c.cursor_shape
    }
    normal->hover(r.enter)
    hover->normal(r.leave)
    hover->drag(r.press)
    drag->hover(r.release)
  }

  FSM resize {
    State idle
    State change {
      Double off_x (0)
      Double off_y (0)
      Double init_w (0)
      Double init_h (0)
      pick.press.x =: off_x
      pick.press.y =: off_y
      r.width =: init_w
      r.height =: init_h
      pick.move.x - off_x + init_w => r.width
      pick.move.y - off_y + init_h => r.height
    }
    idle->change(pick.press)
    change->idle(pick.release)
  }

  FillColor _ (Black)
  Text _ (5, 15, "Drag the rectangle")
  Text _ (5, 30, "Change its size by stretching the bottom right corner")
  Text _ (5, 45, "Change the rounding of the corners using the wheel on the rectangle")
}
