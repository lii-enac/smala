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

  _DEBUG_SEE_COLOR_PICKING_VIEW = 1

  Frame f ("3 layers", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  // Layer BigL {

  //Component LRed {
  Layer LRed (0, 0, 300, 300) {
    Translation _ (0, 0)
    FillColor _ (Red)
    Rectangle r_red (10, 10, 280, 280, 0, 0)

    r_red.press -> (r_red) {
      r_red.width = r_red.width - 5
      r_red.height = r_red.height - 5
    }
  }

  //Component LGreen {
  Layer LGreen (0, 300, 300, 300) {
    Translation _ (0, 300)
    FillColor _ (Green)
    Rectangle r_green (10, 10, 280, 280, 0, 0)

    r_green.press -> (r_green) {
      r_green.width = r_green.width - 5
      r_green.height = r_green.height - 5
    }
  }

  //Component LBlue {
  Layer LBlue (300, 0, 300, 300) {
    Translation _ (300, 0)
    FillColor _ (Blue)
    Rectangle r_blue (10, 10, 280, 280, 0, 0)

    r_blue.press -> (r_blue) {
      r_blue.width = r_blue.width - 5
      r_blue.height = r_blue.height - 5
    }
  }

  //Component Clock {
  Layer Lclock (300, 300, 300, 300) {
    Translation _ (300, 300)
    OutlineColor _ (Red)
    NoFill _
    Rectangle r_clock (10, 10, 280, 280, 0, 0)

    WallClock wc
    FillColor _ (White)
    Text text_clock (50, 50, "default")
    wc.state_text =:> text_clock.text
    
    Clock clock_trigger (250)
    clock_trigger.tick -> wc.state_text

  }

  //} Layer BigL


}
