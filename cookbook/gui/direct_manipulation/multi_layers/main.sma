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
*    
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use display
use gui

_main_
Component root {

  // _DEBUG_SEE_COLOR_PICKING_VIEW = 0
  // _DEBUG_SEE_RECOMPUTE_PIXMAP_AND_PAINTEVENT = 0
  _DEBUG_SEE_RECOMPUTE_PIXMAP_ONLY = 1

  Frame f ("3 layers", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  Layer LRed (0, 0, 0, 0) {
    Translation _ (0, 0)
    FillColor _ (Red)
    Rectangle r_red (10, 10, 280 , 280, 0, 0)
    FillColor _ (Orange)
    Circle r_orange (0, 0, 60)
    r_red.width =:> r_orange.cx
    r_red.height =:> r_orange.cy

    r_red.press -> (r_red) {
      r_red.width = r_red.width - 5
      r_red.height = r_red.height - 5
    }
  }
  f.width / 2 =: LRed.w
  f.height/ 2 =: LRed.h
  
  Layer LGreen (0, 300, 300, 300) {
    Translation t (0, 300)
    FillColor _ (Green)
    Rectangle r_green (10, 10, 280, 280, 0, 0)

    r_green.press -> (r_green) {
      r_green.width = r_green.width - 5
      r_green.height = r_green.height - 5
    }
  }
  
  Layer LBlue (300, 0, 300, 300) {
    Translation t (300, 0)
    FillColor _ (Blue)
    Rectangle r_blue (10, 10, 280, 280, 0, 0)

    r_blue.press -> (r_blue) {
      r_blue.width = r_blue.width - 5
      r_blue.height = r_blue.height - 5
    }
  }

  Layer LBlack (300, 300, 300, 300) {
    Translation t (300, 300)
    OutlineColor _ (Red)
    NoFill _
    Rectangle r_Black (10, 10, 280, 280, 0, 0)
  }

  Component c_clock {
    WallClock wc
    FillColor _ (White)
    Text text_clock (350, 350, "default")
    wc.state_text =:> text_clock.text

    Text text_expl (350, 400, "Clock should not damage Layers")
    
    Clock clock_trigger (250)
    clock_trigger.tick -> wc.state_text
  }
}
