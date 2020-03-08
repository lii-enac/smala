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
 *		Nicolas Saporito <nicolas.saporito@enac.fr>
 *
 */

use core
use exec_env
use base
use display
use gui


_define_
BoolToggleButton (double x, double y, double w, double h, string textContent) {

  /* --- Interface --- */
  Bool value (1) // the boolean value associated to this toggle buutton
  /* - End interface - */

  OutlineColor _ (70, 70, 70)
  FillColor fc_s2 (255, 200, 200)
  Rectangle buttonvalue (x, y, w, h, 5, 5)

  FillColor _ (70, 70, 70)
  TextAnchor _ (1)
  double _x = x + w/2
  double _y = y + h/2 + 5
  Text text (_x, _y, textContent)

  FSM toggleFsm {
    State false {
      255 =: fc_s2.r
      200 =: fc_s2.g
    }
    State true {
      200 =: fc_s2.r
      255 =: fc_s2.g
    }
    false -> true (buttonvalue.press)
    true -> false (buttonvalue.press)
  }
  toggleFsm.state =:> value

}