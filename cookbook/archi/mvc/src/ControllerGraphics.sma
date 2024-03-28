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
ControllerGraphics(Process model, Process view, Process frame)
{ 
  Spike about_to_delete

  Component control {
    // update the view whenever the model changes (subject/observer pattern)
    model.{x,y,width,height} =:> view.r.{x,y,width,height}

    // update model from interactions on the view

    // delta helpers
    Int lastx(0)
    Int dx(0)
    frame.move.x -> {
      frame.move.x - lastx =: dx
      frame.move.x =: lastx
    }
    Int lasty(0)
    Int dy(0)
    frame.move.y -> {
      frame.move.y - lasty =: dy
      frame.move.y =: lasty
    }

    // layout queries: where on the rect did the user press?
    Bool left(0)
    abs(frame.move.x - model.x) <= 5 =:> left
    Bool right(0)
    abs(frame.move.x - (model.x + model.width)) <= 5 =:> right
    Bool top(0)
    abs(frame.move.y - model.y) <= 5 =:> top
    Bool bottom(0)
    abs(frame.move.y - (model.y + model.height)) <= 5 =:> bottom
    Bool center(0)
    not (left || right || bottom || top) =:> center

    Spike left_press    
    Component loffon { frame.press -> left_press }
    left.true -> loffon
    left.false ->! loffon

    Spike right_press    
    Component roffon { frame.press -> right_press }
    right.true -> roffon
    right.false ->! roffon

    Spike top_press    
    Component toffon { frame.press -> top_press }
    top.true -> toffon
    top.false ->! toffon

    Spike bottom_press    
    Component boffon { frame.press -> bottom_press }
    bottom.true -> boffon
    bottom.false ->! boffon

    Spike center_press
    Component coffon { view.r.press -> center_press }
    center.true -> coffon
    center.false ->! coffon


    // actual interactions

    FSM drag {
      State idle
      State dragging {
        Int off_x(0)
        Int off_y(0)
        view.r.press.x - model.x =:  off_x
        view.r.press.y - model.y =:  off_y
        view.r.move.x  - off_x   =:> model.x
        view.r.move.y  - off_y   =:> model.y
      }
      idle->dragging(center_press)
      dragging->idle(frame.release)
    }

    FSM drag_left_border {
      State idle
      State dragging {
        frame.move.x -> {
          model.x + dx =: model.x
          model.width - dx =: model.width
        }
      }
      idle->dragging(left_press)
      dragging->idle(frame.release)
    }
    FSM drag_right_border {
      State idle
      State dragging {
        frame.move.x -> {
          model.width + dx =: model.width
        }
      }
      idle->dragging(right_press)
      dragging->idle(frame.release)
    }
    FSM drag_top_border {
      State idle
      State dragging {
        frame.move.y -> {
          model.y + dy =: model.y
          model.height - dy =: model.height
        }
      }
      idle->dragging(top_press)
      dragging->idle(frame.release)
    }
    FSM drag_bottom_border {
      State idle
      State dragging {
        frame.move.y -> {
          model.height + dy =: model.height
        }
      }
      idle->dragging(bottom_press)
      dragging->idle(frame.release)
    }

    FSM selection {
      State unselected {
        #FF0000 =: view.fill.value
      }
      State selected {
        #00FF00 =: view.fill.value
        GenericKeyboard.key\-pressed == DJN_Key_Delete->about_to_delete
      }
      unselected->selected(view.r.press)
      selected->unselected(view.r.press)
    }
  }


  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this
  }
}