/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use display
use gui

import MyBox
import Arrow

_main_
Component root {
  Frame f ("my frame", 0, 0, 600, 400)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1
  f.background_color.r = 0
  f.background_color.g = 17
  f.background_color.b = 61

  RefProperty cur_box (null)
  RefProperty cur_arrow (null)

  Spike cancel
  cancel->(root) {
    arrow = getRef (root.cur_arrow)
    if (&arrow != null) {
      delete arrow
    }
  }
  FSM fsm_mode {
    State idle {
      cur_box->new_arrow:(root) {
        src = getRef (root.cur_box)
        if (&src != null) {
          addChildrenTo root.arrows {
            Arrow a (src)
            setRef (root.cur_arrow, a)
          }
        } else {
            print ("null src")
        }
      }
    }
    State search_target {
      cur_box->end_arrow:(root){
        dst = getRef (root.cur_box)
        arrow = getRef (root.cur_arrow)
        arrow_src = getRef (arrow.src)
        if (&dst != &arrow_src) {
          setRef (arrow.dst, dst)
          setRef (root.cur_arrow, null)
        } else {
          notify root.cancel
        }
      }
    }
    idle->search_target(idle.new_arrow)
    search_target->idle (search_target.end_arrow)
    search_target->idle (cancel)
  }
  Component boxes {
    MyBox b1 (root, 30, 30)
    MyBox b2 (root, 150, 150)
    MyBox b3 (root, 350, 50)
  }
  Component arrows
}
