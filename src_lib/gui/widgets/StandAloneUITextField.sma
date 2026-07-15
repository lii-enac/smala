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
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base
use display
use gui

import gui.widgets.UITextField

_define_
StandAloneUITextField (int tx_, int ty_, int width_, int height_) inherits UITextField () {
  this.x = tx_
  this.y = ty_
  this.width = width_
  this.height = height_
  this.preferred_width = width_
  this.preferred_height = height_
  this.min_width = width_
  this.min_height = height_
}
