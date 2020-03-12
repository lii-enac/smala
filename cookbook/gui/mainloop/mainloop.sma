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
use base
use display
use gui

_main_
Component root
{
  Frame f ("mainloop", 0, 0, 600, 600)

  FontSize _ (DJN_PX, 14)
  Text t (50, 50, "Drag the rectangle, then close the window.")

  FillColor _ (200, 100, 100)
  Rectangle r (50, 200, 100, 70, 5, 5)

  /* create the connection when the mainloop is started */
  mainloop->(root) {
    addChildrenTo root {
      root.f.move.x => root.r.x
      root.f.move.y => root.r.y
    }
  }

  Spike end_anim
  FSM closing {
    State idle
    State anim {
      Clock cl (60)
      Incr inc (1)
      Incr inc2 (1)
      0 =: inc.state, inc2.state
      cl.tick->inc, inc2
      FillOpacity fo (1)
      OutlineOpacity oo (1)
      FillColor _ (50, 230, 50)
      Circle c (0, 0, 50)
      FillColor _ (0, 0, 0)
      TextAnchor _ (DJN_MIDDLE_ANCHOR)
      Text gb (0, 0, "Good bye!")

      f.width/2 =:> c.cx, gb.x
      f.height/2 =:> c.cy, gb.y
      BoundedValue bv (0, 1, 1)
      inc.state =:> bv.input
      bv.result =:> fo.a, oo.a 
      AssignmentSequence set_pos (1) {
        0.1 =: inc.delta
      }
       AssignmentSequence set_neg (0) {
        -0.1 =: inc.delta
      }
      fo.a == 1 -> set_neg
      fo.a == 0 -> set_pos


      inc2.state >= 70 -> end_anim
    }
    idle->anim (f.close)
    anim->idle (r.press)
  }
  end_anim ->! mainloop

  mainloop !-> (root) {
    delete root
  }
}
