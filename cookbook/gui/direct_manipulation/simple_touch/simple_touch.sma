/*
*  Simple touch app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2018)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*
*/

use core
use base
use gui

_main_
Component root
{
  Frame f ("my frame", 0, 0, 1000, 1000)
  OutlineWidth ow(10)
  FillColor fc(255,0,0)
  OutlineColor _(0,0,255)
  
  Circle mobile (100, 100, 40)
  mobile.touches.$added-> (root) {
    t = getRef (root.mobile.touches.$added)
    addChildrenTo root {
      Component connector {
        t.x => root.mobile.cx
        t.y => root.mobile.cy
      }
    }
  }
  mobile.touches.$removed-> (root) {
    delete root.connector
  }
}

run root
run syshook
