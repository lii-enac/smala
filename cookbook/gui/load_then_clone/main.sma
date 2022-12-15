/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2018)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Poirier <mathieu.poirier@enac.fr>
 *
 */

use core
use base
use display
use gui

 _main_
 Component root
 {

  Frame f ("clone", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  Translation _ (300, 10)

  // OR to load only once (without clone) 
  svg_once = load_from_XML_once ("img/aircraft.svg")
  aircraft_once << svg_once.background
  label_once << svg_once.label

  Translation _ (-300, 10)

  // to load then clone many times the same svg
  for (int i = 0 ; i < 10; i++) {
    Translation _ ( 10 , 10)
    Component aircraft {
      svg = load_from_XML ("img/aircraft.svg")
      background << svg.background
      label << svg.label
    }
  }
}

