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
