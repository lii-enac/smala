/*
 *	StripBoard app
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2020)
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
import StripBoard

_main_
Component root {
  
  Frame f ("my frame", 0, 0, 400, 400)
  mouseTracking = 1

  NoOutline _()
  FillColor _(50, 50, 50)
  Rectangle bkg (0, 0, 400, 400, 0, 0)
  f.width =:> bkg.width
  f.height =:> bkg.height

  Button add (f, "add", 2, 2)
  Button del (f, "del", 2, 33)

  Button asc (f, "asc", 55, 2)
  Button desc (f, "desc", 55, 33)

  StripBoard board (root, 0, 70)
  add.click->board.add
  del.click->board.del
  asc.click->board.ascending
  desc.click->board.descending
}

run root
run syshook
