/*
*  Smala cookbook stylus_and_touches
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

import FingerView

_main_
Component root {
  
  mouseTracking = 0 

  Frame f ("stylus and touches", 0, 0, 600, 600)
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


  //activate touches
  _ENABLE_TOUCHES = 1


  NoFill _
  OutlineColor _ (100,100,255)
  OutlineWidth _ (10)
  OutlineOpacity _ (0.5)

  // The trick here is to store the views in a component, naming them
  // after the unique ID of the touch they are associated with. 
  // We can then implicitly use find/find_optional (without error returns)
  // to leverage the component's internal map and retrieve the 
  // view corresponding to a touch.
  Component touch_views_map

  f.touches.$added->(root) {
    t = getRef (&root.f.touches.$added)
    Process newview = FingerView (root.touch_views_map, toString(t.id) ,t)
    // print ("\n---- added touch")
    // dump t.id
  }
  f.touches.$removed->set_release_touch:(root) {
    t = getRef (&root.f.touches.$removed)
    // print ("\n--- removed touch")
    // dump t.id
    p = find_optional (root.touch_views_map, toString(t.id))
    delete p
  }

}
