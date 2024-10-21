/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use display
use gui

_main_
Component root {
  
  mouseTracking = 0 

  Frame f ("stylus from frame", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex
  Translation t(0,0)

  Component stylus_pen_circ {
    Translation t(0,0)
    OutlineWidth _ (10)
    FillColor _ (255,0,0)
    OutlineColor _(0,0,255)
    Circle c (0,0, 50)
    FillColor _ (White)
    Text _ (0, 0, "PEN")
  }

  FSM fsm_pen {
    State idle
    State drag {
      f.pen.move.x =:> stylus_pen_circ.t.tx
      f.pen.move.y =:> stylus_pen_circ.t.ty
      f.pen.pressure * 50 + 25 =:> stylus_pen_circ.c.r
    }
    idle -> drag (f.pen.press)
    drag -> idle (f.pen.release)
  }

  Component stylus_eraser_circ {
    Translation t(0,0)
    OutlineWidth _ (10)
    FillColor _ (Orange)
    OutlineColor _(0,0,255)
    Circle c (0,0, 50)
    FillColor _ (White)
    Text _ (0, 0, "ERASER")
  }

  FSM fsm_eraser {
    State idle
    State drag {
      f.eraser.move.x =:> stylus_eraser_circ.t.tx
      f.eraser.move.y =:> stylus_eraser_circ.t.ty
      f.eraser.pressure * 50 + 50 =:> stylus_eraser_circ.c.r
    }
    idle -> drag (f.eraser.press)
    drag -> idle (f.eraser.release)
  }
}
