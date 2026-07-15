/*
*  Smala cookbook stylus_shape
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base
use display
use gui

_main_
Component root {
  
  mouseTracking = 1 // to activate enter/leave event 

  Frame f ("stylus on shape", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex
  Translation t(0,0)

  Component stylus_pen_circ {
    Translation t(100,100)
    OutlineWidth ow (10)
    FillColor _ (255,0,0)
    OutlineColor _(0,0,255)
    Circle c (0,0, 50)
    FillColor _ (White)
    Text _ (0, 0, "PEN")
  }

  FSM fsm_pen {
    State idle
    State drag {
      stylus_pen_circ.c.pen.move.x =:> stylus_pen_circ.t.tx
      stylus_pen_circ.c.pen.move.y =:> stylus_pen_circ.t.ty
      stylus_pen_circ.c.pen.pressure * 50 + 50 =:> stylus_pen_circ.c.r
    }
    idle -> drag (stylus_pen_circ.c.pen.press)
    drag -> idle (f.pen.release)
  }

  FSM fsm_pen_over {
    State idle {
      10 =: stylus_pen_circ.ow.width
    }
    State over {
      20 =: stylus_pen_circ.ow.width
    }
    idle -> over (stylus_pen_circ.c.pen.enter)
    over -> idle (stylus_pen_circ.c.pen.leave)
  }

  Component stylus_eraser_circ {
    Translation t(200,200)
    OutlineWidth ow (10)
    FillColor _ (Orange)
    OutlineColor _(0,0,255)
    Circle c (0,0, 50)
    FillColor _ (White)
    Text _ (0, 0, "ERASER")
  }

  FSM fsm_eraser {
    State idle
    State drag {
      stylus_eraser_circ.c.eraser.move.x =:> stylus_eraser_circ.t.tx
      stylus_eraser_circ.c.eraser.move.y =:> stylus_eraser_circ.t.ty
      stylus_eraser_circ.c.eraser.pressure * 50 + 50 =:> stylus_eraser_circ.c.r
    }
    idle -> drag (stylus_eraser_circ.c.eraser.press)
    drag -> idle (f.eraser.release)
  }

  FSM fsm_eraser_over {
    State idle {
      10 =: stylus_eraser_circ.ow.width
    }
    State over {
      20 =: stylus_eraser_circ.ow.width
    }
    idle -> over (stylus_eraser_circ.c.eraser.enter)
    over -> idle (stylus_eraser_circ.c.eraser.leave)
    // You need to add those line below if you want to react when the stylus is hovering over,
    // but not touching, the tablet, as these are registered as mouse events.
    idle -> over (stylus_eraser_circ.c.enter)
    over -> idle (stylus_eraser_circ.c.leave)
  }

  //debug
  Component mouse_circ {
    Translation t(400,400)
    OutlineWidth ow (10)
    FillColor _ (Green)
    OutlineColor _(0,0,255)
    Circle c (0,0, 50)
    FillColor _ (White)
    Text _ (0, 0, "MOUSE")
  }

  FSM fsm_mouse {
    State idle
    State drag {
      mouse_circ.c.move.x =:> mouse_circ.t.tx
      mouse_circ.c.move.y =:> mouse_circ.t.ty
    }
    idle -> drag (mouse_circ.c.press)
    drag -> idle (f.release)
  }

  FSM fsm_mouse_over {
    State idle {
      10 =: mouse_circ.ow.width
    }
    State over {
      20 =: mouse_circ.ow.width
    }
    idle -> over (mouse_circ.c.enter)
    over -> idle (mouse_circ.c.leave)
  }
}
