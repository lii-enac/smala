/*
 *  StripBoard app
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2020)
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
use animation

_define_
Strip (Process holder, int _time, int _position)
{
  Int time (_time)
  Int position (_position)
  Int old_position (_position)

  Translation pos (0, _position * 45)
  x aka pos.tx
  y aka pos.ty

  Spike about_to_delete
  /****************************/
  /*                          */
  /*        Animation         */
  /*  Apparition/disparition  */
  /*                          */
  /****************************/
  FillOpacity fo (0)
  OutlineOpacity oo (0)
  fo.a =:> oo.a
  Incr inc_anim (1)
  Bool end_anim (0)
  FSM fsm_appearing {
    State appearing {
      Clock cl (30)
      SlowInSlowOutInterpolator siso
      0.1 =: inc_anim.delta
      cl.tick->inc_anim
      inc_anim.state=:>siso.input
      siso.output =:> fo.a
      inc_anim.state >= 1 =:> end_anim
    }
    State visible {
      1 =: fo.a
    }
    State disappearing {
      Clock cl (30)
      SlowInSlowOutInterpolator siso
      -0.1 =: inc_anim.delta
      cl.tick->inc_anim
      inc_anim.state=:>siso.input
      siso.output =:> fo.a
      inc_anim.state <= 0 =:> end_anim
    }
    State hidden {
      0 =: fo.a
    }
    appearing->visible (end_anim.true)
    appearing->visible (holder.add)
    visible->disappearing (about_to_delete)
    disappearing->hidden (end_anim.true, holder.ready_to_delete)
    disappearing->hidden (holder.add, holder.ready_to_delete)
  }

  /****************************/
  /*                          */
  /*        Animation         */
  /*  Changement de position  */
  /*                          */
  /****************************/
  Switch anim_pos (idle) {
    Component idle {
      position =: old_position
      position * 45 =: y
    }
    Component anim {
      Double delta (0)
      SlowInSlowOutInterpolator siso ()
      position > old_position ? ((position * 45-  old_position * 45) / 10) : -1 * ((old_position * 45 - position * 45) / 10) =: delta
      holder.count_step_anim.state =:> siso.input
      old_position * 45 + siso.output * delta * 10 =:> y
    }
  }
  holder.strip_animation.state =:> anim_pos.state

  Spike deselect
  Spike select
  Spike insertion_select
  Spike insertion_deselect

  Int deselected (#E57150)
  Int selected (#509AB3)
  Int border_hover (#BC9884)
  Int border_selected (#5072E5)

  NoOutline _
  
  Component border {
    FillOpacity op (0)
    FillColor bkgColor (#BC9884)
    Rectangle r (0, 0, 400, 5, 0, 0)
    Incr inc_anim (1)
    Bool end_anim (0)
    BoundedValue bv (0,1,0)
    inc_anim.state =:> bv.input
    bv.result =:> op.a
    FSM sel {
      State idle {
        0 =: op.a
      }
      State enter {
        0.1 =: inc_anim.delta
        Clock cl (20)
        cl.tick -> inc_anim
        inc_anim.state >= 1 =:> end_anim
        AssignmentSequence set_request (1) {
          this =: holder.insertion_point_select
        }
        r.press->set_request
      }
      State leave {
        -0.1 =: inc_anim.delta
        Clock cl (20)
        cl.tick -> inc_anim
        inc_anim.state <= 0 =:> end_anim
      }
      State hover {
        border_hover =: bkgColor.value
        AssignmentSequence set_request (1) {
          this =: holder.insertion_point_select
        }
        r.press->set_request
      }
      State selected {
        1 =: op.a
        border_selected =: bkgColor.value
        AssignmentSequence set_request (1) {
          this =: holder.insertion_point_deselect
        }
        r.press->set_request
      }
      State leave_selected {}
      {leave,idle}->enter (r.enter)
      enter->hover (end_anim.true)
      {hover,enter}->leave (r.leave)
      enter->selected (insertion_select)
      leave->idle (end_anim.true)
      hover->selected (insertion_select)
      selected->leave_selected (r.leave)
      leave_selected->selected (r.enter)
      leave_selected->idle (insertion_deselect)
      selected->hover (r.press)
      {leave_selected, selected}->idle (holder.insertion_point_deselect_all)
    }
  }


  FillColor bkgColor(#509AB3)
  Rectangle r (0, 5, 400, 40, 0, 0)
  FillColor txt_col (Black)
  Text time_txt (10, 25, "")
  time =:> time_txt.text


  FSM sel {
    State st_deselected {
      deselected =: bkgColor.value
      AssignmentSequence set_request (1) {
        this =: holder.selection_request
      }
      r.press -> set_request
    }
    State st_selected {
      selected =: bkgColor.value
      AssignmentSequence set_request (1) {
        this =: holder.deselection_request
      }
      r.press -> set_request
    }
    st_deselected->st_selected (select)
    st_selected->st_deselected (deselect)
  }
}