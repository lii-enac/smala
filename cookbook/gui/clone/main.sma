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
  Group p1 {
    Rectangle rect1 (100, 100, 100, 100, 0, 0)
    Circle cric1 (300, 100, 50)
  }

  /* clone the all group */
  Translation t1 (0, 200)
  dolly1 << clone (&p1)

  /* clone only circle */
  Translation t2 (0, 200)
  FillColor  _ (255, 0, 0)
  dolly2 << clone (&dolly1.cric1)

  /* test with SVG */
  svg = loadFromXML ("img/aircraft.svg")

  Translation t3 (300, -300)
  Group aircraft {  
    background << svg.background
    label << svg.label
  }

  Translation t4 (0, 200)
  aircraft_clone << clone (&aircraft)
  aircraft_clone.label.text = "CLONE" 

}

