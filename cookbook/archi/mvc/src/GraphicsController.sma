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
GraphicsController(Process model, Process _view, Process frame)
{ 
  Spike about_to_delete
  view aka _view

  //_DEBUG_SEE_COMPONENTS_DESTRUCTION_INFO_LEVEL = 2

  Component control {
    // -- update the view whenever the model changes (subject/observer pattern)
    model.{x,y,width,height} =:> view.r.{x,y,width,height}

    // -- update model from interactions on the view

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
    
                  
                     (abs(px - model.x) <= border) =:> left
     (abs(px - (model.x + model.width)) <= border) =:> right
                     (abs(py - model.y) <= border) =:> top
    (abs(py - (model.y + model.height)) <= border) =:> bottom
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
    delete this
  }
}



/*
    Spike left_press
    // Component loffon(0) { press -> left_press }
    // left.true -> loffon
    // left.false ->! loffon
    FSM loffon {
      State idle
      State in { press -> left_press }
      idle -> in (left.true)
      in -> idle (left.false)
    }

    Spike right_press
    // Component roffon(1) { press -> right_press }
    // right.true -> roffon
    // right.false ->! roffon
    FSM roffon {
      State idle
      State in {
        press -> right_press
      }
      idle -> in (right.true)
      in -> idle (right.false)
    }

    Spike top_press
    // Component toffon(1) { press -> top_press }
    // top.true -> toffon
    // top.false ->! toffon
    FSM toffon {
      State idle
      State in {
        press -> top_press
      }
      idle -> in (top.true)
      in -> idle (top.false)
    }

    Spike bottom_press
    // Component boffon(1) { press -> bottom_press }
    // bottom.true -> boffon
    // bottom.false ->! boffon
    FSM boffon {
      State idle
      State in {
        press -> bottom_press
      }
      idle -> in (bottom.true)
      in -> idle (bottom.false)
    }

    Spike center_press
    // Component coffon(1) { view.r.press -> center_press }
    // center.true -> coffon
    // center.false ->! coffon
    FSM coffon {
      State idle
      State in {
        press -> center_press
      }
      idle -> in (center.true)
      in -> idle (center.false)
    }
*/

    // actual interactions

    // FlipFlop _ (frame.release, frame.press) {
    //   {}
    //   {dx +=> model.width}
    // }
/*
    FSM drag {
      State idle
      State dragging {
        dx +=> model.x
        dy +=> model.y
      }
      idle -> dragging (center_press)
      dragging -> idle (frame.release)
    }

    // Component drag (1) {
    //   dx +=> model.x
    //   dy +=> model.y
    // }
    // center_press -> drag
    // frame.release ->! drag

    FSM drag_left_border {
      State idle
      State dragging {
          dx +=> model.x
        - dx +=> model.width
      }
      idle -> dragging (left_press)
      dragging -> idle (frame.release)
    }
    FSM drag_right_border {
      State idle
      State dragging {
          dx +=> model.width
      }
      idle -> dragging (right_press)
      dragging -> idle (frame.release)
    }
    FSM drag_top_border {
      State idle
      State dragging {
          dy +=> model.y
        - dy +=> model.height
      }
      idle -> dragging (top_press)
      dragging -> idle (frame.release)
    }
    FSM drag_bottom_border {
      State idle
      State dragging {
          dy +=> model.height
      }
      idle -> dragging (bottom_press)
      dragging -> idle (frame.release)
    }
    */
    
    
    
    /*Bool in_rect(0)
        (px >= (model.x - border)) && (px <= (model.x + model.width + border))
    &&  (py >= (model.y - border)) && (py <= (model.y + model.height + border))
    =:> in_rect
    // note: in_rect is required if the user is allowed to click outside the shape, inside the border
    // otherwise use rect.press instead of frame.press
    //Bool in_rect(1)

    // layout queries: where on the rect did the user press? which border, or center?
    Bool left(0)
                     in_rect && 
                     (abs(px - model.x) <= border) =:> left
    */