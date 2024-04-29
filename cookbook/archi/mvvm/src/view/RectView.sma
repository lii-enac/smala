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
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/

use core
use base
use gui

import Handle

_define_
RectView (Process _view_model, Process _frame) {
  vm aka _view_model

  //TextPrinter tp

  int BORDER = 10

  FillColor fill (#FF0000)

  Translation pos (0, 0)

  Rectangle r (0, 0, 0, 0)

  // update the view whenever the view model changes
  //vm.x =:> r.x
  //vm.y =:> r.y
  vm.x =:> pos.tx
  vm.y =:> pos.ty
  vm.width =:> r.width
  vm.height =:> r.height

  // Drag from center
  Double off_x (0)
  Double off_y (0)

  FSM fsm {
    State st_idle {
      #FF0000 =: fill.value
      0 =: vm.is_presssed
    }

    State st_press {
      #EE0000 =: fill.value
      1 =: vm.is_presssed

      //r.press.x - r.x =: off_x
      //r.press.y - r.y =: off_y
      r.press.x - pos.tx =: off_x
      r.press.y - pos.ty =: off_y
      //"offset: " + off_x + " - " + off_y =: tp.input
    }

    State st_dragging {
      #DD0000 =: fill.value

      r.move.x - off_x =:> vm.x
      r.move.y - off_y =:> vm.y
      //"move: " + r.move.x + " - " + r.move.y =: tp.input
    }

    st_idle -> st_press (r.press)
    st_press -> st_idle (r.release)
    st_press -> st_dragging (r.move.x)
    //st_press -> st_dragging (r.move.y)
    st_dragging -> st_idle (r.release)
  }
  
  // Drag with handles on borders
  Handle left (_frame, 0, BORDER, BORDER, 0)
  Handle top (_frame, BORDER, 0, 0, BORDER)
  Handle right (_frame, 0, BORDER, BORDER, 0)
  Handle bottom (_frame, BORDER, 0, 0, BORDER)

  // Update layout of handles
  vm.height - (2 * BORDER) =:> left.height, right.height
  vm.width - (2 * BORDER) =:> top.width, bottom.width  
  vm.width - BORDER =:> right.x
  vm.height - BORDER =:> bottom.y


  // update view model from interactions on the view

  // left.d_x -> {
  //   vm.x + left.d_x =?: vm.x
  //   vm.width - left.d_x =?: vm.width
  // }
  left.d_x +=> vm.x
  -left.d_x +=> vm.width

  // top.d_y -> {
  //   vm.y + top.d_y =?: vm.y
  //   vm.height - top.d_y =?: vm.height
  // }
  top.d_y +=> vm.y
  -top.d_y +=> vm.height

  // right.d_x -> {
  //   vm.width + right.d_x =?: vm.width
  // }
  right.d_x +=> vm.width

  // bottom.d_y -> {
  //   vm.height + bottom.d_y =?: vm.height
  // }
  bottom.d_y +=> vm.height

}