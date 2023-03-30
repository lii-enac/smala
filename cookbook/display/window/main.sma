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
  Frame f ("transparent windows", 0, 0, 800, 600)
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

  Component Explanation {
    Translation _ (100, 10)
    FillColor _ (Black)
    FillOpacity _ (0.8)
    Rectangle _ (0, 0, $f.width - 200 , 120, 0, 0)
    FillColor w (White)
    Text l1 (10, 20,  "note:")
    Text l2 (10, 40,  "- For a transparent window you HAVE to set f.frameless = 1")
    Text l3 (10, 60,  "- All transparent windows will be set  \"On Top\" of all other windows")
    Text l4 (10, 80,  "- Windows: only fully transparent windows are available BUT you CAN click through the window")
    Text l5 (10, 100, "- Linux/macos: opacity and color on transparency are available BUT you CAN'T click through the window")
  }

  Component Group {
    Translation _ (100, 100)
    FillColor g (0, 255, 0)
    Rectangle r (0, 60, 100, 100)
    FillColor _ (Black)
    Text _ (10, 80, "Click me")
    

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