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
GraphicsPresentation(Process abstraction, Process frame) {
  Component view {
    FillColor fill (Red)
    Rectangle r (0,0,0,0)
  }

  Component control {
    // -- update the view whenever the abstraction changes (subject/observer pattern)
    abstraction.{x,y,width,height} =:> view.r.{x,y,width,height}

    // -- update abstraction from interactions on the view

    //press aka frame.press
    press aka view.r.press
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

    Int border(5)

    // layout queries: where on the rect did the user press? which border, or center?
    Bool center(0)
    Bool left(0)
    Bool right(0)
    Bool top(0)
    Bool bottom(0)
    
                  
                     (abs(px - abstraction.x) <= border) =:> left
     (abs(px - (abstraction.x + abstraction.width)) <= border) =:> right
                     (abs(py - abstraction.y) <= border) =:> top
    (abs(py - (abstraction.y + abstraction.height)) <= border) =:> bottom
              not (left || right || bottom || top) =:> center

    // mini state-machines to synthesize transient events upon press on borders or center
    Spike center_press
    Spike left_press
    Spike right_press
    Spike top_press
    Spike bottom_press

    FSM area_press {
      State idle
      State in_center { press -> center_press }
      State in_left   { press -> left_press }
      State in_right  { press -> right_press }
      State in_top    { press -> top_press }
      State in_bottom { press -> bottom_press }
      
      idle -> in_center   (center.true)
      in_center -> idle   (center.false)
      idle -> in_left  (left.true)
      in_left -> idle  (left.false)
      idle -> in_right (right.true)
      in_right -> idle (right.false)
      idle -> in_top   (top.true)
      in_top -> idle   (top.false)
      idle -> in_bottom   (bottom.true)
      in_bottom -> idle   (bottom.false)
    }

    // actual interactions and abstraction updates
    FSM drag {
      State idle
      State dragging_center {
          dx +=> abstraction.x
          dy +=> abstraction.y
      }
      State dragging_left {
          dx +=> abstraction.x
        - dx +=> abstraction.width
      }    
      State dragging_right {
          dx +=> abstraction.width
      }
      State dragging_top {
          dy +=> abstraction.y
        - dy +=> abstraction.height
      }
      State dragging_bottom {
          dy +=> abstraction.height
      }
      idle -> dragging_center (center_press)
      idle -> dragging_left (left_press)
      idle -> dragging_right (right_press)
      idle -> dragging_top (top_press)
      idle -> dragging_bottom (bottom_press)
      dragging_center, dragging_left, dragging_right, dragging_top, dragging_bottom -> idle (frame.release) // why {} ???
    }

  }
}