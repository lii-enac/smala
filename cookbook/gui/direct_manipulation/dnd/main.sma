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

import Mobile

_main_
Component root
{
  Frame f ("my frame", 0, 0, 1000, 1000)
  Exit ex (0, 1)
  f.close->ex
  OutlineWidth ow(10)
  FillColor fc(255,0,0)
  OutlineColor _(0,0,255)

  RefProperty selected (null)
  Spike s_delete
  s_delete->(selected) {
    obj = getRef (&selected)
    if (&obj != null) {
      delete obj
      setRef (&selected, null)
    }
  }

  Component trash {
    FillColor fc_trash (#3E4245)
    Int dark (#3E4245)
    Int light (#838C91)
    Rectangle icon(500,500,300,100)
    FSM fsm_delete {
      State out {
        dark =: fc_trash.value
      }
      State in {
        light =: fc_trash.value
        icon.release->s_delete
      }
      out->in (icon.enter)
      in->out (icon.leave)
    }
  }
  Mobile mob (selected, f, 100, 100) 
}
