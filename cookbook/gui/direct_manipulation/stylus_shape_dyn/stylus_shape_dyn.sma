/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023-2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui

_main_
Component root
{
  //activate touches
  _ENABLE_TOUCHES = 1
  
  Frame f ("Stylus shape dynamic memorie", 0, 0, 1000, 1000)
  Exit ex (0, 1)
  f.close -> ex
  Incr incr (0)
  incr.delta = 1

  FillColor _ (White)
  Text text_counter (20, 20, "")
  "Clock Tick: " + incr.state =:> text_counter.text

  Component content {

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
    }

  }

  Clock cl (5000)
  cl.tick -> incr
  cl.tick -> (root) {
    delete_content root.content

    addChildrenTo root.content {
      
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
        drag -> idle (root.f.pen.release)
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
        drag -> idle (root.f.eraser.release)
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
      }
    } 
  }

}
