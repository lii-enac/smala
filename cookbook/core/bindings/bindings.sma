/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2019)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use display
use gui

//import Button
import gui.widgets.StandAlonePushButton

_main_
Component root {
  Frame f ("my frame", 0, 0, 1200, 600)
  Exit ex (0, 1)
  f.close -> ex
  StandAlonePushButton show ("Show", 50, 50)
  StandAlonePushButton hide ("Hide", 50, 150)

  FillColor _(255, 0, 0)
  Rectangle rectR (50, 250, 100, 70, 0, 0)
  show.click -> rectR
  hide.click ->! rectR

  FillColor _ (0, 255, 0)
  Rectangle rectG (250, 250, 100, 70, 0, 0)
  rectR !-> rectG
  rectR ->! rectG

  FillColor _ (0, 0, 255)
  Rectangle rectB (450, 250, 100, 70, 0, 0)
  rectR !->! rectB
  show.click -> rectB
}
