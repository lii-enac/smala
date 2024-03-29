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
Dial (Process frame, double _x, double _y) {
  Translation t (_x, _y)
  x aka t.tx
  y aka t.ty

  gdial = load_from_XML ("img/dial.svg")

  /*---- interface -----*/
  Double input (0)
  Spike edit
  Spike exec
  /*----- end interface -----*/
  

  Component gobj {
    Translation translate (0, 0)
    Rotation rot (0, 0, 0)
    Scaling scale (1, 1, 0, 0)
    needle << gdial.needle
    gdial.axis_mark.cx =:> rot.cx
    gdial.axis_mark.cy =:> rot.cy
  }

  Double initWidth (0)
  Double initHeight (0)
  initWidth = gdial.shadow.bkg.width
  initHeight = gdial.shadow.bkg.height

  FSM fsm {
    State st_exec {
      input * 360 =:> gobj.rot.a
    }

    State st_edit {
      shadow << gdial.shadow

      Double zero (0)
      zero =: gobj.rot.a

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

    st_exec->st_edit (edit)
    st_edit->st_exec (exec)
  }
}
