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
*    
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*/

use core
use base
use gui

import gui.interactors.SimpleDrag

_main_
Component root {
  Frame f ("simplest", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1

  FillColor bg (50, 50, 50)
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.width =:> bkg.width
  f.height =:> bkg.height

  Ref null_ref (null)
  Ref toDrag (null)
  AssignmentSequence set_null (1) {
    null_ref =: toDrag
  }

  SimpleDrag _ (toDrag, f)
  List my_scene

  bkg.press->(root) {
    addChildrenTo root.my_scene {
      Component new_cpnt {
        FillColor _ (70, 200, 70)
        Rectangle r (10, 10, 100, 70, 5, 5)
        press aka r.press
        enter aka r.enter
        leave aka r.leave
        x aka r.x
        y aka r.y
      }
    }
  }

  my_scene.$added -> (root) {
    new_cpnt = getRef(root.my_scene.$added)
    addChildrenTo root {
      new_cpnt.enter -> { new_cpnt =: root.toDrag }
      new_cpnt.leave -> { root.null_ref =: root.toDrag }
    }
  }
  
  toDrag->(toDrag) {
    src = getRef (toDrag)
    if (&src != null) {
      moveChild src >>
    }
  }
}
