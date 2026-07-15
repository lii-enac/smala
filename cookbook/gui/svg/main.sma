/*
*  Smala cookbook svg
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use gui

import gui.interactors.PanAndZoom

_main_
Component root {

  _ENABLE_TOUCHES = 1

  Frame frame ("svg", 100, 100, 1000, 500)
  frame.close ->! mainloop

  // see the screenshots of the illustrator SVG export dialog boxes 
  // string path = "http://smala.io/button.svg"
  string path = "simple_rect.svg"
  svg = load_from_XML_once (path, "myattr, edgeNameFromXPlane, pathLength")
  //dump svg

  t << svg

  TextPrinter tp 
  "_SVG_USER_CUSTOM_ATTRS: myattr= " + t.background.myattr + " - edgeNameFromXPlane= " +t.background.edgeNameFromXPlane + " - pathLength= " + t.background.pathLength =:> tp.input
}
