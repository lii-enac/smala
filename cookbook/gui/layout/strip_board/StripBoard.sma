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

import Strip

_define_
StripBoard (Process parent, double _x, double _y)
{
  Ref selection_request (null)
  Ref deselection_request (null)
  Ref insertion_point_select (null)
  Ref insertion_point_deselect (null)

  Spike insertion_point_deselect_all
  Spike end_anim_position
  Spike ready_to_delete

  Spike add
  Spike del

  Spike update_position

  Translation _ (_x, _y)

  List selected_items()
  List strip_list()
  Int position (0)

  Sorter sorter (strip_list, "time")
  Sorter pos_sorter (strip_list, "position")
  sorter.ascending->sorter.sort

  AssignmentSequence ascending (1) {
    1 =: sorter.ascending
  }
  AssignmentSequence descending (1) {
    0 =: sorter.ascending
  }

  /***************************/
  /*                         */
  /*          Sort           */
  /*                         */
  /***************************/
  sorter.sort->l_sort:(this) {
    for (int i = 1; i <= $this.strip_list.size; i++) {
      this.strip_list.[i].position = i - 1
    }
    notify this.update_position
  }

  l_sort~>this.update_position
  /***************************/
  /*                         */
  /*  Selection/deselection  */
  /*                         */
  /***************************/

  selection_request->(this) {
    selected = getRef (&this.selection_request)
    if (&selected != null) {
      addChildrenTo this.selected_items {
        Ref new_ref (selected)
      }
      notify selected.select
    }
  }

  deselection_request->(this) {
    item = getRef (&this.deselection_request)
    if (&item != null) {
      notify item.deselect
      for (int i = 1; i <= $this.selected_items.size; i++) {
        sel_item = getRef (&this.selected_items.[i])
        if (&sel_item == &item) {
          delete this.selected_items.[i]
        }
      }
    }
  }

  insertion_point_select->(this) {
    selected = getRef (&this.insertion_point_select)
    if (&selected != null) {
      notify selected.insertion_select
      this.position = selected.position
      for (int i = 1; i <= $this.strip_list.size; i++) {
        item = &this.strip_list.[i]
        if (&item != &selected) {
          notify item.insertion_deselect
        }
      }
    }
  }

  insertion_point_deselect->(this) {
    item = getRef (&this.insertion_point_deselect)
    if (&item != null) {
      notify item.insertion_deselect
    }
    this.position = this.strip_list.size - 1
  }
  /***************************/
  /*                         */
  /*    Addition/deletion    */
  /*                         */
  /***************************/
  Int time (0)

  add->insertion_point_deselect_all
  del->insertion_point_deselect_all

  Bool test_anim (0)
  position == strip_list.size =:> test_anim

  FSM with_anim_fsm {
    State without_anim {
      add->(this) {
        addChildrenTo this.strip_list {
          Strip s (this, $this.time, $this.position)
        }
        this.time++
        this.position = this.strip_list.size
      }
    }
    State with_anim {
      add->(this) {
        notify this.pos_sorter.sort
        for (int i = $this.position + 1; i <= $this.strip_list.size; i++) {
          this.strip_list.[i].position++
        }
        this.time++
        notify this.update_position
      }
      end_anim_position->(this) {
        addChildrenTo this.strip_list {
          Strip s (this, $this.time, $this.position)
        }
        this.time++
        this.position = this.strip_list.size
      }
    }
    without_anim->with_anim (test_anim.false)
    with_anim->without_anim (test_anim.true)
  }

  del->(this) {
    for (int i = 1; i <= $this.selected_items.size; i++) {
      selected = getRef (&this.selected_items.[i])
      if (&selected != null) {
        notify selected.about_to_delete
        
      }
    }
  }

  ready_to_delete->(this) {
    for (int i = 1; i <= $this.selected_items.size; i++) {
      selected = getRef (&this.selected_items.[i])
      delete selected 
      setRef (&this.selected_items.[i], null)
    }
    for (int j = 1; j <= $this.selected_items.size; j++) {
      delete this.selected_items.[j]
    }
    notify this.pos_sorter.sort
    for (int k = 1; k <= $this.strip_list.size; k++) {
      this.strip_list.[k].position = k - 1
    }
    this.position = $this.strip_list.size
    notify this.update_position
  }

  /***************************/
  /*                         */
  /*        Animation        */
  /*                         */
  /***************************/
  Incr count_step_anim (1)
  count_step_anim.delta = 0.1
  Bool end_anim (0)

  count_step_anim.state > 1 =:> end_anim
  FSM strip_animation {
    State idle
    State anim {
      Clock cl (30)
      0 =: count_step_anim.state
      cl.tick -> count_step_anim
    }
    idle->anim (this.update_position)
    anim->idle (end_anim.true, end_anim_position)
    anim->idle (add, end_anim_position)
    anim->idle (del, end_anim_position)
  }
}