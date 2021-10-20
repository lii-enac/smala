/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2021)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
 
use core
use base
use gui

_define_
DraggableCircle (int _x, int _y, int _r)
{
  FillColor _(Red)
  Circle c (_x, _y, _r)
  cx aka c.cx
  cy aka c.cy
  r aka c.r
  release aka c.release
  press aka c.press
  Int req_x (0)
  Int req_y (0)
  FSM fsm {
    State idle
    State dragging {
      Double off_x (0)
      Double off_y (0)
      c.press.x - c.cx =: off_x
      c.press.y - c.cy =: off_y
      c.move.x - off_x =:> req_x
      c.move.y - off_y =:> req_y
    }
    idle->dragging (c.press)
    dragging->idle (c.release)
  } 
}