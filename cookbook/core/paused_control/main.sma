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
 *    Nicolas Saporito  <nicolas.saporito@enac.fr>
 *
 */

use core
use base
use gui

_main_
Component root {
  Frame f ("paused_control", 0, 0, 500, 600)
  Exit ex (0, 1)
  f.close -> ex

  FillColor fc (0, 0, 0)
  Text t_prev_prev (20, 120, "0")
  Text t_prev (100, 120, "0")
  Text t_cur (180, 120, "0")

  Incr incr (1)
  
  // sequence of assignments to activate on mouse press
  // (0 = lazy mode -> not activated when instanciated)
  AssignmentSequence seq (0) {
    t_prev.text =: t_prev_prev.text
    t_cur.text =: t_prev.text
  }
  f.press -> seq

  // data flow (no need to sequence)
  t_prev.text -> incr       // everytime the sequence ends, activate incr...
  incr.state => t_cur.text // ...and connect the increment value to current text...
  // this connection can also be made without propagation (::>) in order to avoid a potential loop

}

run root
run syshook