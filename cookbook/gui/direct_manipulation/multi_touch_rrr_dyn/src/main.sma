/*
*  Multitouch drag
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2019)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Mathieu poirier <mathieu.poirier@enac.fr>
*
*/

use core
use base
use gui
use display

import TouchMarker

_main_
Component root
{
  Frame frame ("Multitouch Drag", 0, 1000, 1000, 1000)

  Incr incr (0)
  incr.delta = -10
  incr.state = 255

  Component content {
    FillColor fc($incr.state,0,0)
    OutlineColor _(0,0,$incr.state)

    Component tt {
      Homography h
      Rectangle r (400, 400, 150, 150, 0, 0)
    }

    Component tt2 {
      Homography h
      Rectangle r (200, 200, 150, 150, 0, 0)
    }
    ScaleRotateTranslate _ (tt.r, tt.h)
    ScaleRotateTranslate _ (tt2.r, tt2.h)
  }

  TouchMarker tm (frame)

  Clock cl (5000)
  cl.tick -> incr
  cl.tick -> (root) {
    delete root.content

    addChildrenTo root {
       Component content {
         FillColor fc($root.incr.state,0,0)
         OutlineColor _(0,0,$root.incr.state)

         Component tt {
           Homography h
           Rectangle r (400, 400, 150, 150, 0, 0)
         }

         Component tt2 {
            Homography h
           Rectangle r (200, 200, 150, 150, 0, 0)
         }
         ScaleRotateTranslate _ (tt.r, tt.h)
         ScaleRotateTranslate _ (tt2.r, tt2.h)
       }
    } 
  }

}

run root
run syshook
