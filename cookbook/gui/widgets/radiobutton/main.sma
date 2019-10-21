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
use display
use gui

import Radiobutton

_main_
Component root {
  Frame f ("my frame", 0, 0, 400, 200)
  Exit ex (0, 1)
  f.close -> ex

  Radiobutton rb (5)
  rb.entries.1.label = "a"
  rb.entries.2.label = "b"
  rb.entries.3.label = "c"
  rb.entries.4.label = "d"
  rb.entries.5.label = "e"

  FillColor fc (0, 0, 0)
  Incr incr (1)  
  Text t (150, 50, "")
  isString ("Selected entry: " + rb.entry + " state: " + incr.state) =:> t.text

  rb.entries.2.selected -> incr
}

run root
run syshook
