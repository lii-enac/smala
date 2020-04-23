/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Stéphane Conversy <stephane.conversy@enac.fr>
 *    Mathieu Poirier <mathieu.poirier@enac.fr>
 *
 */


use core
use base
use display
use gui

_define_
Mobile (Process ref, Process f, int x, int y)
{
  Translation t (x, y)
  Circle circ (0, 0, 40)
  FSM fsm {
    State idle {
      1 =: circ.pickable
    }
    State dragging {
      this =: ref
      0 =: circ.pickable
      Int off_x (0)
      Int off_y (0)
      f.move.x - t.tx =: off_x
      f.move.y - t.ty =: off_y 
      f.move.x - off_x =:> t.tx
      f.move.y - off_y =:> t.ty
    }
    idle -> dragging (circ.press)
    dragging -> idle (f.release)
  }
}
