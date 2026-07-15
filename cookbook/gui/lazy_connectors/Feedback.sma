/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2025)
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

_define_
Feedback (int _tx, int _ty  , int _time)
{ 
  Translation t (_tx, _ty)

  Spike trigger

  FSM fsm {
    State off {

    }
    State on {
      FillColor _ (Green)
      Rectangle rec (0, 0, 15, 15, 0, 0)
      Timer timer (_time)
    }

    off -> on (trigger)
    on -> off (fsm.on.timer.end)
  }
  


}
