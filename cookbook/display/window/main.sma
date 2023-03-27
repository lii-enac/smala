/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2021)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use display
use gui

_main_
Component root {
  Frame f ("transparent windows", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex


  // important note: 
  // for a full transparent frame you HAVE to set f.frameless = 1
  f.frameless = 1
  f.background_opacity = 0.3
  f.background_color.r = 0
  f.background_color.g = 0
  f.background_color.b = 255

  //then not transparent component
  Component rect_close {
    FillColor _ (Red)
    Rectangle r (0, 0, 20, 20)
  }
  rect_close.r.press -> ex

  Component Group {
    Translation _ (100, 100)
    FillColor w (White)
    Text txt (0, 30, "for a full transparent frame you HAVE to set f.frameless = 1")
    FillColor g (0, 255, 0)
    Rectangle r (0, 60, 100, 100)
    

    FSM fsm {
      State idle {
      }
      State circle {
        FillColor g (Yellow)
        Circle  c (200, 200, 50)
      }

      idle -> circle (r.press)
      circle -> idle (r.release)
    }
  }

  
}