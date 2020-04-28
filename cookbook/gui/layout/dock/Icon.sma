/*
 *  dock app
 *
 *  The copyright holders for the contents of this file are:
 *  Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *  Contributors:
 *     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use gui

_define_
Icon (Process holder, Process _prev, int _x, int _y)
{
  Translation pos (_x, _y)
  Ref prev (_prev)
  x aka pos.tx
  y aka pos.ty

  if (&_prev != null) {
    Component pos_cnt {
      _prev.x + _prev.width =:> x
    }
  }
  
  prev->(this) {
    prev = getRef (&this.prev)
    if (&prev != null) {
      addChildrenTo this {
        Component pos_cnt {
          prev.x + prev.width =:> this.x
        }
      }
    }
  }

  Translation _ (10, 0)
  Scaling sc (1, 1, 0, 45)
  Rotation rot (0, 22.5, 22.5)
  Rectangle r (0, 0, 45, 45, 0, 0)

  Double width (45)
  (r.width * sc.sx) + 10  =:> width

  Incr inc (1)
  1 + inc.state =:> sc.sx, sc.sy
  Spike end_anim ()
  FSM behavior {
    State idle {
      0 =: inc.state
      1 =: sc.sx, sc.sy
    }
    State enlarge {
      Clock cl (20)
      0.1 =: inc.delta
      cl.tick -> inc
      (inc.state >= 1) -> end_anim
    }
    State large {
      2 =: sc.sx, sc.sy
      1 =: inc.state
    }
    State reduce {
      Clock cl (20)
      -0.1 =: inc.delta
      cl.tick -> inc
      (inc.state <= 0) -> end_anim
    }
    {idle, reduce}->enlarge (r.enter)
    enlarge->large (end_anim)
    {enlarge, large}->reduce (r.leave)
    reduce->idle (end_anim)
  }

  FSM del_fsm {
    State idle {
      0 =: rot.a
    }
    State anim {
      Clock cl (150)
      FSM fsm {
        State left {
          -5 =: rot.a
        }
        State right {
          5 =: rot.a
        }
        left->right (cl.tick)
        right->left (cl.tick)
      }
      AssignmentSequence set_del (1) {
        this =: holder.to_delete
      }
      r.press->set_del
    }
    idle->anim (holder.del)
    anim->idle (holder.del)
    anim->idle (holder.add)
  }
}