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
*    Sébastien Leriche <sebastien.leriche@enac.fr>
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

  f.touches.$added-> (root) {
    t = getRef (root.f.touches.$added)
    addChildrenTo root {
      Circle fingerInt (100, 100, 40)
      Circle fingerExt (100, 100, 50)
      fingerInt.cx => fingerExt.cx
      fingerInt.cy => fingerExt.cy
      Component fingerConnector {
        t.x => root.fingerInt.cx
        t.y => root.fingerInt.cy
      }
    }
  }

  f.touches.$removed-> (root) {
    delete root.fingerConnector
  }
}

run root
run syshook
