/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/

use core
use base
use gui

_define_
Handle (Process _frame, double _x, double _y, double _width, double _height) {
  frame aka _frame

  TextPrinter tp

  NoOutline _
  FillColor fill (#888888)

  Rectangle r (_x, _y, _width, _height)
  x aka r.x
  y aka r.y
  width aka r.width
  height aka r.height

  //"Handle " + x + " " + y +  " -- w " + width + " -- h " + height =:> tp.input

  Double previous_move_x (0)
  Double previous_move_y (0)

  Double d_x (0)
  Double d_y (0)

  FSM fsm {
    State st_idle {
      #888888 =: fill.value
      //0 =: vm.is_presssed
    }

    State st_press {
      #666666 =: fill.value
      //1 =: vm.is_presssed

      r.press.x =: previous_move_x
      r.press.y =: previous_move_y
      //"Press: " + previous_move_x + " - " + previous_move_y =: tp.input
    }

    State st_dragging {
      #444444 =: fill.value

      //"drag: " + d_x + " - " + d_y => tp.input

      frame.move -> {
        frame.move.x - previous_move_x =: d_x
        frame.move.y - previous_move_y =: d_y

        frame.move.x =: previous_move_x
        frame.move.y =: previous_move_y
      }
    }

    st_idle -> st_press (r.press)
    st_press -> st_idle (r.release)
    st_press -> st_dragging (frame.move.x)
    //st_press -> st_dragging (frame.move.y)
    st_dragging -> st_idle (r.release)
  }

}