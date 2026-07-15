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
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base
use gui
use display

import display.display

_define_
AltKey (Process f, int key)
{
  Spike press
  Spike release
  
  Bool isAltPress (0)
  Bool isAltRelease (0)
  f.key\-pressed == DJN_Key_Alt =:> isAltPress
  f.key\-released == DJN_Key_Alt =:> isAltRelease

  FSM fsm {
    State off
    State on {
      f.key\-pressed == key -> press
      f.key\-released == key -> release
    }
    off->on (isAltPress.true)
    on->off (isAltRelease.true)
  }
}
