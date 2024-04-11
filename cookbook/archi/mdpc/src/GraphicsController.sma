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
*/

use core
use base
use gui

_define_
GraphicsController(Process model, Process _display_view, Process _picking_view, Process frame)
{ 
  Spike about_to_delete
  view aka _display_view
  picking_view aka _picking_view

  //_DEBUG_SEE_COMPONENTS_DESTRUCTION_INFO_LEVEL = 2

  Component control {
    // -- update the views whenever the model changes (subject/observer pattern)
    model.{x,y,width,height} =:> view.r.{x,y,width,height}
    model.{x,y,width,height} =:> picking_view.r.{x,y,width,height}

    // -- update model from interactions on the view

    //press aka frame.press
    press aka picking_view.r.press
    move aka frame.move
    //release aka frame.release
    px aka frame.move.x
    py aka frame.move.y
    
    // delta helpers
    Int lastx(0)
    Int dx(0)
    px -> {
      px - lastx =: dx
              px =: lastx
    }
    Int lasty(0)
    Int dy(0)
    py -> {
      py - lasty =: dy
              py =: lasty
    }
    frame.release -> {
      0 =: dx
      0 =: dy
    }

    center_press aka picking_view.r.press
    left_press aka picking_view.left.press
    right_press aka picking_view.right.press
    top_press aka picking_view.top.press
    bottom_press aka picking_view.bottom.press

    // actual interactions and model updates
    FSM drag {
      State idle
      State dragging_center {
          dx +=> model.x
          dy +=> model.y
      }
      State dragging_left {
          dx +=> model.x
        - dx +=> model.width
      }    
      State dragging_right {
          dx +=> model.width
      }
      State dragging_top {
          dy +=> model.y
        - dy +=> model.height
      }
      State dragging_bottom {
          dy +=> model.height
      }
      idle -> dragging_center (center_press)
      idle -> dragging_left (left_press)
      idle -> dragging_right (right_press)
      idle -> dragging_top (top_press)
      idle -> dragging_bottom (bottom_press)
      { dragging_center, dragging_left, dragging_right, dragging_top, dragging_bottom } -> idle (frame.release) // why {} ???
    }

  }

  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this.picking_view
    delete this
  }
}
