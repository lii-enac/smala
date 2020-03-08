/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use exec_env
use base
use display
use gui

import Button
import ToggleButton
import Slider
import Dial

_main_
Component root {
  Frame f ("my frame", 100, 100, 400, 600)
  Exit ex (0, 1)
  f.close -> ex

  mouseTracking = 1

  Button exit (f, "Exit", 10, 15)
  exit.click->ex

  ToggleButton edit (f, "Edit", 325, 15)
  f.width - edit.getwidth - 10 =:> edit.x

  Slider s (f, 100, 250)
  Dial d (f, 200, 150)

  s.output =:> d.input
  edit.on -> d.edit, s.edit
  edit.off -> d.exec, s.exec

}

run root
run syshook
