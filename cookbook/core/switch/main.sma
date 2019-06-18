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


_main_
Component root {
  Frame f ("f", 0, 0, 500, 300)
  Exit ex (0, 1)
  f.close -> ex

  /* style */
  FillColor bg (70, 151, 255)
  OutlineColor oc (240, 240, 240)
  OutlineWidth w (2)
  /* shape */
  Rectangle r (10, 10, 200, 110, 5, 5)

  /* behavior */
  Switch sw (greater) {
    Component greater {
      f.width / 2 =:> r.x	
    }
    Component lower {
    	  Double x (10)
    	  x =: r.x
    }
  }
  f.width > 400 ? "greater" : "lower" =:> sw.state
}

run root
run syshook
