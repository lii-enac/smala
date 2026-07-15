/*
*  Smala cookbook sprite_animation
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui

_main_
Component root {
  Frame f ("sprite animation", 0, 0, 400, 400)
  Exit ex (0, 1)
  f.close -> ex
  
  
  Translation tx (100, 100)

  FillOpacity _ (0)

  frame1_svg = load_from_XML_once ("Frame1.svg")
  frame2_svg = load_from_XML_once ("Frame2.svg")
  frame3_svg = load_from_XML_once ("Frame3.svg")
  frame4_svg = load_from_XML_once ("Frame4.svg")

  Clock cl (400)

  SwitchList anim {
    Component f1 {
      frame1 << frame1_svg
    }
    Component f2 {
      frame2 << frame2_svg
    }
    Component f3 {
      frame3 << frame3_svg
    }
    Component f4 {
      frame4 << frame4_svg
    }
  }
  cl.tick -> anim.next
  1 =: anim.loop

}
