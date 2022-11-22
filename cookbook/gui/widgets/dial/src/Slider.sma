/*
*	djnn Smala compiler
*
*	The copyright holders for the contents of this file are:
*		Ecole Nationale de l'Aviation Civile, France (2017)
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

_define_
Slider (Process frame, double _x, double _y) {
  Translation t (_x, _y)

  x aka t.tx
  y aka t.ty

  gslider = loadFromXML ("img/slider.svg")

  /*----- interface -----*/
  Double input (0)
  Spike edit
  Spike exec
  /*----- end interface -----*/

  Component gobj {
    Translation translate (0, 0)
    Scaling scale (1, 1, 0, 0)
    slider << gslider.slider
    Translation t_thumb (0, 0)
    thumb << gslider.thumb
  }

  Double initWidth (0)
  Double initHeight (0)
  initWidth = gslider.shadow.bkg.width
  initHeight = gslider.shadow.bkg.height

  Double output (0)
  -gobj.t_thumb.ty / 275 =:> output
  Double dy (0)
  Double min (-275.0)
  Double max (0)
  Double buff (0)

  
  dy > min ? dy : min =:> buff
  buff < max ? buff : max =:> gobj.t_thumb.ty
  

  FSM fsm {
    State st_edit {
      shadow << gslider.shadow
      FSM dragUpLeft {
        State idle {
          FillColor grey (157, 157, 157)
          grey.r =: shadow.upLeft.fill.r, shadow.upLeft.stroke.r
          grey.g =: shadow.upLeft.fill.g, shadow.upLeft.stroke.g
          grey.b =: shadow.upLeft.fill.b, shadow.upLeft.stroke.b
        }
        State enter {
          FillColor red (255, 0, 0)
          red.r =: shadow.upLeft.fill.r, shadow.upLeft.stroke.r
          red.g =: shadow.upLeft.fill.g, shadow.upLeft.stroke.g
          red.b =: shadow.upLeft.fill.b, shadow.upLeft.stroke.b
        }
        State dragging {
          Double offX (0)
          Double offY (0)
          
          shadow.upLeft.press.y - shadow.upLeft.y =: offY
          shadow.upLeft.press.x - shadow.upLeft.x =: offX
          frame.move.y - offY =:> shadow.upLeft.y
          frame.move.x - offX =:> shadow.upLeft.x
          
          shadow.bkg.width / initWidth =:> gobj.scale.sx
          shadow.bkg.height / initHeight =:> gobj.scale.sy
          (initWidth - shadow.bkg.width) / 2  =:> gobj.translate.tx
          (initHeight - shadow.bkg.height) / 2  =:> gobj.translate.ty
          
          shadow.upLeft.x + 2.5 =:> shadow.bkg.x
          shadow.upLeft.x =:> shadow.downLeft.x
          shadow.upRight.x - shadow.upLeft.x =:> shadow.bkg.width
          shadow.upLeft.y + 2.5 =:> shadow.bkg.y
          shadow.upLeft.y =:> shadow.upRight.y
          shadow.downLeft.y - shadow.upLeft.y =:> shadow.bkg.height
          
          shadow.bkg.x + (shadow.bkg.width / 2) - 2.5 =:> shadow.middle.x
          shadow.bkg.y + (shadow.bkg.height / 2) - 2.5 =:> shadow.middle.y
          
        }
        idle->enter (shadow.upLeft.enter)
        enter->dragging (shadow.upLeft.press)
        dragging->enter (shadow.upLeft.release)
        enter->idle (shadow.upLeft.leave)
        dragging->idle (frame.release)
      }
      FSM dragMiddle {
        State idle
        State dragging {
          Double offX (0)
          Double offY (0)
          Double initTx (0)
          Double initTy (0)
          t.tx =: initTx
          t.ty =: initTy
          shadow.middle.press.y =: offY
          shadow.middle.press.x =: offX
          frame.move.y - offY + initTy =:> t.ty
          frame.move.x - offX + initTx =:> t.tx
        }
        idle->dragging (shadow.middle.press)
        dragging->idle (frame.release)
      }
    }
    State st_exec {
      FSM exec_fsm {
        State idle {
          gobj.t_thumb.ty =: dy
        }
        State dragging {
          Double offset (0)
          gobj.thumb.press.y - (gobj.scale.sy * dy) =: offset
          (initHeight /fsm.st_edit.shadow.bkg.height)  * (frame.move.y - offset) =:> dy
        }
        idle->dragging (gobj.thumb.press)
        dragging->idle (frame.release)
      }
    }
    st_exec->st_edit (edit)
    st_edit->st_exec (exec)
  }
  fsm.initial = "st_exec"
}
