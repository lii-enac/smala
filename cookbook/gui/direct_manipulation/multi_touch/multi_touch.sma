/*
*  Multitouch touch app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2018-2019)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Sébastien Leriche <sebastien.leriche@enac.fr>
*    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*
*/

use core
use base
use gui
use display

_main_
Component root
{
  Frame f ("my frame", 0, 0, 1000, 1000)
  NoFill _
  Dictionary d_touch
  OutlineColor _ (100,100,255)
  OutlineWidth _ (10)
  OutlineOpacity _ (0.5)

  f.touches.$added->(root) {
    t = getRef (root.f.touches.$added)
    addChildrenTo root {
      Component fingerConnector {
        Circle finger (-100, -100, 100)
        t.x => root.fingerConnector.finger.cx
        t.y => root.fingerConnector.finger.cy
      }
    setRef (root.d_touch.key, t)
    setRef (root.d_touch.value, fingerConnector)
    run root.d_touch.add
    }
  }
  f.touches.$removed->(root) {
    t = getRef (root.f.touches.$removed)
    setRef (root.d_touch.key, t)
    p = getRef (root.d_touch.value)
    run root.d_touch.delete
    delete p
  }
}

run root
run syshook
