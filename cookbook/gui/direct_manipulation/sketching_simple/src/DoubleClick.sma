/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2018-2021)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Nicolas Saporito <nicolas.saporito@enac.fr>
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base

import APoint
import Distance

_define_
/* 
 * Double click management:
 * mouseTracking must be enabled in main app
 * shape must have press and release children 
 */
DoubleClick (Process f, Process shape)
{
  int CLICK_DELAY = 500
 
  Spike double_click

  Bool max_distance (0)
  APoint p1 (0, 0)
  APoint p2 (0, 0)
  Distance d (p1, p2)
  d.result > 10 =:> max_distance
 
  FSM fsm {
    State idle

    State first_press {
      Timer t (CLICK_DELAY)
      shape.press.x =: p1.x, p2.x
      shape.press.y =: p1.y, p2.y
    }

    State first_release {
      Timer t (CLICK_DELAY)
      f.move.x =:> p2.x
      f.move.y =:> p2.y
    }

    idle -> first_press (shape.press)

    first_press -> first_release (shape.release)
    first_press -> idle (first_press.t.end)

    first_release -> idle (shape.press, double_click)
    first_release -> idle (first_release.t.end)
    first_release -> idle (max_distance.true)

  }
}