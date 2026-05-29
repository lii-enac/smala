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
  Frame f ("Outline Width", 0, 0, 600, 800)
  
  Exit ex (0, 1)
  f.close -> ex

  TextPrinter tp

  FillColor _ (#444444)
  OutlineColor _ (#FF0000)

  Component left {
    OutlineWidth _ (0.00)
    Rectangle _ (10, 10, 90.47, 70.53)

    OutlineWidth _ (0.01)
    Rectangle _ (10, 110, 90.47, 70.53)

    OutlineWidth _ (0.25)
    Rectangle _ (10, 210, 90.47, 70.53)

    OutlineWidth _ (0.5)
    Rectangle _ (10, 310, 90.47, 70.53)

    OutlineWidth _ (0.75)
    Rectangle _ (10, 410, 90.47, 70.53)

    OutlineWidth _ (1.0)
    Rectangle _ (10, 510, 90.47, 70.53)

    OutlineWidth _ (2)
    Rectangle _ (10, 610, 90.47, 70.53)

    OutlineWidth _ (3)
    Rectangle _ (10, 710, 90.47, 70.53)
  }

  Component center {
    Translation tr (140, 0)

    FillColor labels_color (#FFFFFF)

    Text _ (0, 50, "Outline width = 0.00")
    Text _ (0, 150, "Outline width = 0.01")
    Text _ (0, 250, "Outline width = 0.25")
    Text _ (0, 350, "Outline width = 0.5")
    Text _ (0, 450, "Outline width = 0.75")
    Text _ (0, 550, "Outline width = 1.0")
    Text _ (0, 650, "Outline width = 2.0")
    Text _ (0, 750, "Outline width = 3.0")
  }

  Component right {
    Translation tr (320, 0)

    Component labels {
      FillColor labels_color (#FFFFFF)
      FontWeight _ (DJN_BOLD)

      Text _ (0, 470, "Use mouse wheel to update outline width:")
      Text txt (0, 490, "")
    }

    OutlineWidth ow (1.0)
    Rectangle r (0, 510, 90.47, 70.53)

    // To check that there is no impact by previous OutlineWidth
    OutlineWidth _ (2)
    Rectangle _ (0, 610, 90.47, 70.53)

    Double dynamic_ow (1)

    Pow p (1.01, 0)
    f.wheel.dy =:> p.exponent

    AssignmentSequence seq (0) {
      dynamic_ow * p.result =: dynamic_ow
      dynamic_ow =: ow.width
      ow.width =: labels.txt.text
    }
    p.result -> seq

    r.press -> {
      0.01 =: dynamic_ow
      0.0 =: ow.width
    }
    
    "wheel " + f.wheel.dy + " -- pow " + p.result + " -- ow " + dynamic_ow =:> tp.input
  }

}
