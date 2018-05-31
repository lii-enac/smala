/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
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
use gui

import Checkbox

_main_
Component root {
  Frame f ("my frame", 0, 0, 400, 200)
  Exit ex (0, 1)
  f.close -> ex

  Checkbox cb (f, 5)
  cb.entries.1.label = "a"
  cb.entries.2.label = "b"
  cb.entries.3.label = "c"
  cb.entries.4.label = "d"
  cb.entries.5.label = "e"

  FillColor fc (0, 0, 0)
  Incr incr (1)
  String s = "Selected entry: " + cb.entry + " state: " + incr.state 
  Text t (150, 50, "")
  s => t.text

  cb.entries.2.selected -> incr
}

run root
run syshook
