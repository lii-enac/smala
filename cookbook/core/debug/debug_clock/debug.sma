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

  Frame f ("f", 0, 0, 800, 800)
  Exit ex (0, 1)
  f.close -> ex
  //mouseTracking = 1
  //FillColor fc(255,0,0)
  Rectangle r (50, 50, 100, 50, 0, 0)
  
  FSM fsm {
	  State idle
	  State clock {
		  Clock cl (500)
		  Incr incr (1)
		  cl.tick->incr
		  TextPrinter tp
		  incr.state=:>tp.input
	  }
	  idle->clock (r.press)
	  clock->idle (r.press)
  }
}
