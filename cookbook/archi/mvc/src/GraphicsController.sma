/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023-2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*
*/
use core
use base
use gui

_define_
GraphicsController(Process model, Process _view, Process frame)
{ 
  Spike about_to_delete
  view aka _view

  Component control {
    // -- observe the model and update the view whenever the model changes (subject/observer pattern)
    // 'src =:> dst' setups a data-flow: it both observes the src and updates the dst
    model.{x,y,width,height} =:> view.r.{x,y,width,height}


    // -- update model from interactions on the view

    // abstract events away 
    press aka view.r.press
    move aka frame.move
    release aka frame.release
    px aka frame.move.x
    py aka frame.move.y
    
    // move delta helpers // FIXME should be in move?
    Double lastx(0)
    Double dx(0)
    px -> {
      px - lastx =: dx
              px =: lastx
    }
    Double lasty(0)
    Double dy(0)
    py -> {
      py - lasty =: dy
              py =: lasty
    }

    // query layout of view internals: where on the rect did the user press? which border, or center?
    Int border(5)

    Bool center(0)
    Bool left(0)
    Bool right(0)
    Bool top(0)
    Bool bottom(0)
                  
                      (abs(px - view.r.x) <= border) =:> left
     (abs(px - (view.r.x + view.r.width)) <= border) =:> right
                      (abs(py - view.r.y) <= border) =:> top
    (abs(py - (view.r.y + view.r.height)) <= border) =:> bottom
              not (left || right || bottom || top) =:> center

    // synthesize events upon press on borders or center
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
      
      idle -> in_center  (center.true)
      in_center -> idle  (center.false)
      idle -> in_left    (left.true)
      in_left -> idle    (left.false)
      idle -> in_right   (right.true)
      in_right -> idle   (right.false)
      idle -> in_top     (top.true)
      in_top -> idle     (top.false)
      idle -> in_bottom  (bottom.true)
      in_bottom -> idle  (bottom.false)
    }

    // control interactions and update model
    // 'control' means: 'control the state of the interaction' (the FSM) and 'activate data-flows' (inside states)
    // 'update' is implemented with an addition connector '+=>'
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
      idle -> dragging_left   (left_press)
      idle -> dragging_right  (right_press)
      idle -> dragging_top    (top_press)
      idle -> dragging_bottom (bottom_press)
      { dragging_center, dragging_left, dragging_right, dragging_top, dragging_bottom } -> idle (release) // FIXME: why {} ???
    }

  }

  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this
  }
}
