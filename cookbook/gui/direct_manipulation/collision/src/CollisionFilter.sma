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
use display
use gui

_native_code_
%{
  using namespace djnn;
  using namespace std;
  #include <cmath>
%}

_action_
fn_update_value (Process src, Process data)
{
  int req_cx = data.req_cx.value
  int req_cy = data.req_cy.value
  int r = data.r.value

  current = getRef (data.current)
  int collision = 0
  for item:data.my_list {
    if (&item != &current) {
      double dst = sqrt ( (item.cx - req_cx)*(item.cx - req_cx) +  (item.cy - req_cy)*(item.cy - req_cy))
      if (dst < (item.r + current.r)) {
        collision = 1
        break
      }
    }
  }
  if (collision == 0) {
    data.cx.value = req_cx
    data.cy.value = req_cy
  }
}

_define_
CollisionFilter (Process _list, Process _current) {
  DerefDouble cx (&_current, "cx", DJNN_GET_ON_CHANGE)
  DerefDouble cy (&_current, "cy", DJNN_GET_ON_CHANGE)
  DerefDouble r (&_current, "r", DJNN_GET_ON_CHANGE)

  DerefDouble req_cx (&_current, "req_x", DJNN_SET_ON_CHANGE)
  DerefDouble req_cy (&_current, "req_y", DJNN_SET_ON_CHANGE)

  Deref release (&_current, "release")
  
  my_list aka _list
  current aka _current
  FSM fsm {
    State idle
    State dragging {
      NativeAction update_value (fn_update_value, this, 1)
      req_cx.value -> update_value
      req_cy.value -> update_value
    }
    idle->dragging (_current)
    dragging->idle (release.activation)
  }
}
