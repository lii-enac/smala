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

  Component control {
    // -- update the views whenever the model changes (subject/observer pattern)
    // and transform the model into a display and picking views
    model.{x,y,width,height} =:> view.{x,y,width,height}
    model.{x,y,width,height} =:> picking_view.{x,y,width,height}

    // -- transform user actions on the view into operations on the model
    
    // delta helpers
    px aka frame.move.x
    py aka frame.move.y
    Double lastx(0)
    Double sdx(0) // screen dx
    px -> {
      px - lastx =: sdx
              px =: lastx
    }
    Double lasty(0)
    Double sdy(0) // screen dy
    py -> {
      py - lasty =: sdy
              py =: lasty
    }

    // inverse transform user actions, from screen to model coordinates
    Double mdx(0) // model dx
    Double mdy(0) // model dy

    ScreenToLocal m (view.r)
    px =:> m.inX
    py =:> m.inY

    ScreenToLocal m2 (view.r)
    px-sdx =:> m2.inX
    py-sdy =:> m2.inY

    m.outX-m2.outX =:> mdx
    m.outY-m2.outY =:> mdy

    dx aka mdx
    dy aka mdy

    // actual interactions and model updates
    press aka picking_view.r.press
    move aka frame.move

    center_press aka picking_view.r.press
    left_press aka picking_view.left.press
    right_press aka picking_view.right.press
    top_press aka picking_view.top.press
    bottom_press aka picking_view.bottom.press

    // the FSM manages the interaction state by activating/deactivating dataflows
    // interactions on picking views manage the FSM transitions
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
      { dragging_center, dragging_left, dragging_right, dragging_top, dragging_bottom } -> idle (frame.release) // FIXME: why {} ???
    }

  }

  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this.picking_view
    delete this
  }
}
