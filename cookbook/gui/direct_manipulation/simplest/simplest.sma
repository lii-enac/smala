/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use exec_env // _main_ loop
use base
use display
use gui

_main_
Component root {
  Frame f ("simplest", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex
  Translation t(0,0)

  OutlineWidth ow(10)
  FillColor fc(255,0,0)
  OutlineColor _(0,0,255)

  Circle _(0,0, 50)

  mouseTracking = 1
  f.move.x =:> t.tx
  f.move.y =:> t.ty
}
