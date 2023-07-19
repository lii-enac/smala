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
use display
use gui


_main_
Component root
{
  //activate touches
  _DEBUG_NO_TOUCH_EVENT = 0

  Frame f ("my frame", 0, 0, 1000, 1000)
  Exit ex (0, 1)
  f.close -> ex
  NoFill _
  Dictionary d_touch
  OutlineColor _ (100,100,255)
  OutlineWidth _ (10)
  OutlineOpacity _ (0.5)

  f.touches.$added->(root) {
    t = getRef (&root.f.touches.$added)
    addChildrenTo root {
      Component fingerConnector {
        Circle finger (-100, -100, 100)
        t.move.x =:> finger.cx
        t.move.y =:> finger.cy
        dump t
      }
    setRef (&root.d_touch.key, &t)
    setRef (&root.d_touch.value, &fingerConnector)
    run root.d_touch.add
    }
  }
  f.touches.$removed->(root) {
    t = getRef (&root.f.touches.$removed)
    setRef (&root.d_touch.key, &t)
    p = getRef (&root.d_touch.value)
    run root.d_touch.del
    delete p
  }
}
