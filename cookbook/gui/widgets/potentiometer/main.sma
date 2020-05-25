/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use display
use gui

import gui.widgets.Potentiometer

_main_
Component root {
  Frame f ("my frame", 0, 0, 300, 300)
  Exit ex (0, 1)
  f.close -> ex

  FillColor _ (0, 0, 0)
  Text t1 (45, 150, "")
  Text t2 (145, 150, "")
  Text t3 (245, 150, "")
  Int i1 (0)
  "R: " + i1 =:> t1.text
  Int i2 (0)
  "G: " + i2 =:> t2.text
  Int i3 (0)
  "B: " + i3 =:> t3.text
  
  Potentiometer pot_r (f, 50, 200, 40)
  Potentiometer pot_g (f, 150, 200, 40)
  Potentiometer pot_b (f, 250, 200, 40)

  FillColor col (0, 0, 0)
  Rectangle r (100, 50, 100, 70, 5, 5)
  pot_r.output * 255 =:> col.r, i1
  pot_g.output * 255 =:> col.g, i2
  pot_b.output * 255 =:> col.b, i3
}
