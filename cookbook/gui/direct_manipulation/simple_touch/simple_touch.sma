/*
*  Smala cookbook simple_touch
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2018-2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     stephane conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui

_main_
Component root
{ 
  // activate touches
  _ENABLE_TOUCHES = 1

  Frame f ("my frame", 0, 0, 1000, 1000)
  Exit ex (0, 1)
  f.close -> ex
  OutlineWidth ow(10)
  FillColor fc(255,0,0)
  OutlineColor _(0,0,255)
  
  //a place to temporary keep the active conector
  Component active_connector

  Circle mobile (100, 100, 40)
  mobile.touches.$added-> (root) {
    t = getRef (&root.mobile.touches.$added)
    addChildrenTo root.active_connector {
      t.move.x =:> root.mobile.cx
      t.move.y =:> root.mobile.cy
    }
  }
  mobile.touches.$removed-> (root) {
    delete_content root.active_connector
  }
}
