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

  selection_request->l_sel_req:(this) {
    selected = getRef (&this.selection_request)
    if (&selected != null) {
      addChildrenTo this.selected_items {
        Ref new_ref (selected)
      }
      notify selected.select
    }
  }

  deselection_request->l_desel_req:(this) {
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

  insertion_point_select->l_ins_sel:(this) {
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

  insertion_point_deselect->l_ins_desel:(this) {
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

  del->l_del:(this) {
    for (int i = 1; i <= $this.selected_items.size; i++) {
      selected = getRef (&this.selected_items.[i])
      if (&selected != null) {
        notify selected.about_to_delete
        
      }
    }
  }

  FSM with_anim_fsm {
    State without_anim {
      add->l_add:(this) {
        addChildrenTo this.strip_list {
          Strip s (this, $this.time, $this.position)
          graph_add_edge (this.l_sel_req, s.select)
          graph_add_edge (this.l_desel_req, s.deselect)
          graph_add_edge (this.l_ins_sel, s.insertion_deselect)
          graph_add_edge (this.l_ins_sel, s.insertion_select)
          graph_add_edge (this.l_ins_desel, s.insertion_deselect)
          graph_add_edge (this.l_del, s.about_to_delete)
          graph_add_edge (this.l_sort, s.position)
        }
        this.time++
        this.position = this.strip_list.size
      }
      l_add~>time
      l_add~>position
    }
    State with_anim {
      add->pos_sorter.sort
      pos_sorter.sort->l_add:(this) {
        for (int i = $this.position + 1; i <= $this.strip_list.size; i++) {
          this.strip_list.[i].position++
        }
      }
      l_add->update_position

      end_anim_position->l_end:(this) {
        addChildrenTo this.strip_list {
          Strip s (this, $this.time, $this.position)
          graph_add_edge (this.l_sel_req, s.select)
          graph_add_edge (this.l_desel_req, s.deselect)
          graph_add_edge (this.l_ins_sel, s.insertion_deselect)
          graph_add_edge (this.l_ins_sel, s.insertion_select)
          graph_add_edge (this.l_ins_desel, s.insertion_deselect)
          graph_add_edge (this.l_del, s.about_to_delete)
          graph_add_edge (this.l_sort, s.position)
        }
        this.time++
        this.position = this.strip_list.size
      }
      l_end~>time
      l_end~>position
    }
    without_anim->with_anim (test_anim.false)
    with_anim->without_anim (test_anim.true)
  }

  ready_to_delete->l_ready:(this) {
    for (int i = 1; i <= $this.selected_items.size; i++) {
      selected = getRef (&this.selected_items.[i])
      graph_remove_edge (this.l_sel_req, selected.select)
      graph_remove_edge (this.l_desel_req, selected.deselect)
      graph_remove_edge (this.l_ins_sel, selected.insertion_deselect)
      graph_remove_edge (this.l_ins_sel, selected.insertion_select)
      graph_remove_edge (this.l_ins_desel, selected.insertion_deselect)
      graph_remove_edge (this.l_del, selected.about_to_delete)
      graph_remove_edge (this.l_sort, selected.position)
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
  l_ready~>pos_sorter.sort
  pos_sorter.sort~>update_position

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