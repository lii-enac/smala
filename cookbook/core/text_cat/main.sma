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
use base
use display
use gui

_main_
Component root {
  Frame f ("f", 0, 0, 500, 600)
  Exit ex (0, 1)
  f.close -> ex
  FillColor fc (255, 0, 0)
  Text t (20, 20, "")
  "frame width =  " + f.width + " frame height = " + f.height =:> t.text
}

