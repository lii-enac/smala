/*
*  Smala Library
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2021-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base

import display.window

import gui.widgets.AbstractBox


_action_
fn_update_items_pos_and_geom (Process src, Process data)
{
  int nb_items = data.items.size
  
  int padding_left = data.padding_left
  int padding_top = data.padding_top
  int preferred_width = data.preferred_width
  int preferred_height = data.preferred_height
  int width  = (preferred_width  != -1 ? preferred_width  : data.container_width)  - 2*padding_left
  int height = (preferred_height != -1 ? preferred_height : data.container_height) - 2*padding_top
  // int preferred_width = -1
  // int preferred_height = -1
  // int width = data.container_width - 2*padding_left
  // int height = data.container_height - 2*padding_top

  int space = data.space

  int fixed_height = 0
  int nb_fixed_height = 0
  for item : data.items {
    if ($item.preferred_height != -1) {
      fixed_height += $item.preferred_height
      nb_fixed_height++
    }
  }

  int space_per_item = 0
  if (nb_items != nb_fixed_height) {
    space_per_item = (height - fixed_height - (nb_items-1)*space - 2*padding_top) / (nb_items - nb_fixed_height)
  }

  int dy = padding_top
  int max_width = 0
  for item : data.items {
    if ($item.preferred_height != -1) {
      item.height = $item.preferred_height
    } else {
      if ($item.max_height != -1) {
        item.height = space_per_item < $item.max_height ? (space_per_item < $item.min_height ? $item.min_height : space_per_item) : $item.max_height
      } else {
        item.height = space_per_item < $item.min_height ? $item.min_height : space_per_item
      }
    }
    item.y = dy
    dy += item.height + space

    if ($item.preferred_width != -1) {
      item.width = $item.preferred_width
    } else {
      item.width = width - 2*padding_left
      if ($item.width > $item.max_width && $item.max_width > -1) {
        item.width = $item.max_width
      } else if ($item.width < $item.min_width) {
        item.width = $item.min_width
      }
    }
    if ($item.width > $max_width) {
      max_width = $item.width
    } 
  }
  if ($data.items.size>0) {
    dy -= space // remove last added space
  }

  for item : data.items {
    if (item.h_alignment == 0) { // left
      item.x = 0
    } else if (item.h_alignment == 1) { // center
      item.x = max_width/2 - item.width/2
    } else {
      item.x = max_width - item.width
    }
  }

  if (data.set_pos == 1) {
    if ($data.h_alignment == 0) {
      data.off_x = 0
    } else if ($data.h_alignment == 1) {
      data.off_x = (data.container_width - max_width) / 2 - padding_left
    } else {
      data.off_x = data.container_width - max_width - padding_left
    }

    if ($data.v_alignment == 0) {
      data.off_y = 0
    } else if ($data.v_alignment == 1) {
      data.off_y = (data.container_height - dy) / 2 - padding_top
    } else {
      data.off_y = data.container_height - dy - padding_top
    }
  }

  if (data.top_level_box != 1) {
    data.preferred_width = max_width + 2*padding_left
    data.preferred_height = dy + 2*padding_top
    data.cell_width = max_width
    data.cell_height = space_per_item
  }
}

_define_
VBox () inherits AbstractBox ()
{ 
  NativeAction update_items_pos_and_geom (fn_update_items_pos_and_geom, this, 0)
  this.container_width->update_items_pos_and_geom
  this.container_height->update_items_pos_and_geom 
  SumList sl (this.items, "min_height")
  MaxList ml (this.items, "min_width")
  sl.output->update_items_pos_and_geom // bad trick to force geometry recomputation at startup, needed for text
  sl.output + this.space*(this.items.size - 1) =:> this.min_height
  ml.output =:> this.min_width
  this.min_width -> update_items_pos_and_geom
  this.min_height -> update_items_pos_and_geom

  if (this.top_level_box == 1) {
    Timer t(1)
    t.end -> this.pack     // FIXME
    t.end -> update_items_pos_and_geom

    if (this.preferred_width == -1) {
      this.container_width =:> this.preferred_width
    }
    if (this.preferred_height == -1) {
      this.container_height =:> this.preferred_height
    }
  }
  this.pack ~> update_items_pos_and_geom
  
  moveChild this.remaining >>
}
