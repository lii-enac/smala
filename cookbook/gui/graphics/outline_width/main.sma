/*
*  Smala cookbook outline_width
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
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
    Rectangle _ (10, 10, 190.47, 70.53)

    OutlineWidth _ (0.01)
    Rectangle _ (10, 110, 190.47, 70.53)

    OutlineWidth _ (0.25)
    Rectangle _ (10, 210, 190.47, 70.53)

    OutlineWidth _ (0.5)
    Rectangle _ (10, 310, 190.47, 70.53)

    OutlineWidth _ (0.75)
    Rectangle _ (10, 410, 190.47, 70.53)

    OutlineWidth _ (1.0)
    Rectangle _ (10, 510, 190.47, 70.53)

    OutlineWidth _ (2)
    Rectangle _ (10, 610, 190.47, 70.53)

    OutlineWidth _ (3)
    Rectangle _ (10, 710, 190.47, 70.53)

    Component left_labels {
      Translation _ (25, 0)

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
  }

  Component right {
    Translation tr (320, 0)

    Component labels {
      FillColor labels_color (#FFFFFF)
      FontWeight _ (DJN_BOLD)

      Text _ (0, 70, "Use mouse wheel to update outline width:")
      Text txt (0, 90, "")

      Text _ (0, 380, "- NO Outline")
      Text _ (0, 400, "- Outline Width (3)")
      Text _ (0, 530, "- Outline Color (Orange)")
      Text _ (0, 550, "- Outline Width (3)")
    }

    OutlineWidth ow (1.0)
    Rectangle r (0, 110, 190.47, 70.53)

    // To check that there is no impact by previous OutlineWidth
    OutlineWidth ow2 (2)
    Rectangle r2 (0, 210, 190.47, 70.53)

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
      "press set 0.0 " =: tp.input
      0 =: ow.width
      ow.width =: labels.txt.text

      0.01 =: dynamic_ow  // Not strictly 0 to allow to continue to use mouse wheel: dynamic_ow * p.result != 0
    }

    r2.left.press -> {
      "left press set -2 " =: tp.input
      -2.0 =: ow2.width
    }

    r2.right.press -> {
      "right press set 2 " =: tp.input
      2.0 =: ow2.width
    }
    
    // "wheel " + f.wheel.dy + " -- pow " + p.result + " -- ow " + dynamic_ow =:> tp.input

    Component right_labels {
      Translation _ (25, 0)

      FillColor labels_color (#FFFFFF)

      Text _ (0, 150, "press: ow = 0")
      
      Text _ (0, 240, " left press: ow = -2")
      Text _ (0, 260, "right press: ow = 2")

    }

    NoOutline _
    OutlineWidth ow3 (3)
    Rectangle r3 (0, 420, 190.47, 70.53)

    OutlineColor _ (#FF8800)
    OutlineWidth ow4 (3)
    Rectangle r4 (0, 570, 190.47, 70.53)
  }

}
