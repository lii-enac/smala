/*
*  Smala Library
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2021)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Magnaudet Mathieu <mathieu.magnaudet@enac.fr>
*
*/
use core
use base
use gui
use display

import display.display

_define_
ControlKey (Process f, int key)
{
  Spike press
  Spike release
  
  Bool isControlPress (0)
  Bool isControlRelease (0)
  f.key\-pressed == DJN_Key_Control =:> isControlPress
  f.key\-released == DJN_Key_Control =:> isControlRelease

  FSM fsm {
    State off
    State on {
      f.key\-pressed == key -> press
      f.key\-released == key -> release
    }
    off->on (isControlPress.true)
    on->off (isControlRelease.true)
  }
}
