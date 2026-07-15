/*
*  Smala cookbook window
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2019-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui

_main_
Component root {
  Frame f ("transparent windows", 0, 0, 1000, 600)
  Exit ex (0, 1)
  f.close -> ex

  // transparency 
  f.background_isTransparent = 1 // will auto set frameless to true

  // plain background without frame
  // f.frameless = 1
  // f.background_color.r = 0
  // f.background_color.g = 0
  // f.background_color.b = 255

  //then not transparent component
  Component rect_close {
    FillColor _ (Red)
    Rectangle r (0, 0, 20, 20)
  }
  rect_close.r.press -> ex

  Component background {
    FillOpacity _ (0.1)
    FillColor _ (Red)
    Rectangle rec (200, 200, 0, 0, 0, 0)
    f.width - 400 =:> rec.width
    f.height - 400 =:> rec.height
  }

  Component Explanation {
    Translation _ (100, 10)
    FillColor _ (Black)
    FillOpacity _ (0.8)
    Rectangle _ (0, 0, $f.width - 200 , 140, 0, 0)
    FillColor w (White)
    Text l1 (10, 20,  "Note:")
    Text l2 (10, 40,  "- Setting 'isTransparent' to true will automatically set 'isFrameless' to true.")
    Text l3 (10, 60,  "- Transparent windows are automatically set to 'Always on Top'.")
    Text l4 (10, 80,  "- Windows are fully transparent and support 'Click-Through' (mouse events pass through).")
    Text l5 (10, 100, "- Real-time updates to background color or transparency are NOT supported")
    Text l6 (10, 120, "- Tip: Use a Rectangle with custom opacity as a child element if semi-transparency is needed.")
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