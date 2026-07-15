/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use gui

_action_
enter_action (Process c)
%{
 	Process *data = (Process*) get_native_user_data (c);
  // data->dump(0);

 	Process *container = data->get_parent ();

  if (container)
    container->move_child (data, LAST, nullptr);

%}


_define_
Item (int _color, int _x, int _y)
{
  /* interface */
  Translation pos (_x, _y)
  x aka pos.tx
  y aka pos.ty

  Int color (_color)

  Spike pressed
  Spike released

  FillColor fc (_color)
  OutlineColor oc (Black)
  FSM fsm_color {
    State st1 {
    }
    State st2 {
    }

    st1 -> st2 (pressed)
    st2 -> st1 (released)
  }

  Switch sw_color ("st1") {
    Component st1 {
      #000000 =: oc.value
    }
    Component st2 {
      #ff0000 =: oc.value
    }
  }
  
  fsm_color.state =:> sw_color.state

  OutlineWidth _ (3)
  Circle c (0, 0, 20)

  NativeAction enter_na (enter_action, this, 1)
  c.press -> pressed 
  c.press -> enter_na
  c.release -> released
}