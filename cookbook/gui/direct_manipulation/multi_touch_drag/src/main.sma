/*
*  Multitouch drag
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2018-2019)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Nicolas Saporito  <nicolas.saporito@enac.fr>
*
*/

use core
use base
use gui
use display

import TouchDrag
import TouchMarker

_main_
Component root
{
  Frame frame ("Multitouch Drag", 0, 1000, 1000, 1000)

  Component r1 {
    Translation initial_position (400, 400)
    Homography transforms

    FillColor _ (255, 100, 100)
    OutlineColor _ (255, 50, 50)
    OutlineWidth _ (2)
    Rectangle rect (0, 0, 200, 150, 0, 0)

    TouchDrag td (frame, rect, transforms)
  }

  Component r2 {
    Translation initial_position (500, 500)
    Homography transforms

    FillColor _ (100, 200, 100)
    OutlineColor _ (50, 200, 50)
    OutlineWidth _ (2)
    Rectangle rect (0, 0, 200, 150, 0, 0)

    TouchDrag td (frame, rect, transforms)
  }

  TouchMarker tm (frame)
}

run root
run syshook
