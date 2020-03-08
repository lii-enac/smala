/*
*  ms_per_frame app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*  Contributors:
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>    
*
*/

use core
use exec_env
use base
use display
use gui

import mspf

_main_
Component root
{
	Frame f ("ms_per_frame", 0, 0, 512, 512)

	OutlineWidth ow(10)
	FillColor fc(255,0,0)
  OutlineColor _(0,0,255)

  Component _ {
  Translation t(0,0)
  
  Circle mobile(100, 100, 40)
  Text _(0,0,"text")
  f.move.x =:> t.tx
  f.move.y =:> t.ty
  }
  FillColor _(0,0,0)
  mspf _(f)
}

run root
run syshook