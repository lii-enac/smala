/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2017)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *    Stéphane Conversy <stephane.conversy@enac.fr>
 *    Nicolas Saporito  <nicolas.saporito@enac.fr>
 *
 */

use core
use base
use display
use gui

_main_
Component root {

  Frame f ("my frame", 0, 0, 600, 800)
  Exit ex (0, 1)
  f.close -> ex

  FillColor fc (0, 0, 0)
  Translation tr (100, 100)
  Scaling sc (2, 2, 0, 0)
  Circle mobile (100, 100, 40)

  Component offset {
    DoubleProperty x (0)
    DoubleProperty y (0)
  }

  FSM fsm {
    State idle

    State waiting_hyst {
      FillColor fcwh (255, 0, 0)
      Circle c (0, 0, 20)
      // take Translation then Scaling into acount to
      // place and dimension the hysteresis Circle
      (f.press.x - tr.tx) / sc.sx =: c.cx
      (f.press.y - tr.ty) / sc.sy =: c.cy
      20 / sc.sx =: c.r
    }

    State dragging {
      waiting_hyst.c.cx * sc.sx =: offset.x
      waiting_hyst.c.cy * sc.sy =: offset.y
      f.move.x - offset.x => tr.tx
      f.move.y - offset.y => tr.ty
    }
    
    idle -> waiting_hyst (mobile.press)
    waiting_hyst -> idle (f.release)
    waiting_hyst -> dragging (waiting_hyst.c.leave)
    dragging -> idle (f.release)
  }

  TextPrinter tp
  fsm.state => tp.input
}

run root
run syshook
