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

_native_code_
%{
#include "core/utils/iostream.h"

inline
void myprint(const djnnstl::string& s)
{
  puts(s.c_str());
}
%}

_action_
fn_update_items_pos_and_geom (Process src, Process data)
{
  // string ssname = get_debug_name(data)
  // print ("--- fn_update_items_pos_and_geom")
  // myprint(ssname)

  int nb_items = data.items.size
  // print("nb_item: " + data.items.size)
  // if (nb_items==0) {
  //   return
  // }

  int padding_left = data.padding_left
  int padding_top  = data.padding_top
  int preferred_width  = data.preferred_width
  int preferred_height = data.preferred_height
  int width  = (preferred_width  != -1 ? preferred_width  : data.container_width)  - 2*padding_left
  int height = (preferred_height != -1 ? preferred_height : data.container_height) - 2*padding_top
  // int preferred_width = -1
  // int preferred_height = -1
  // int width = data.container_width - 2*padding_left
  // int height = data.container_height - 2*padding_top

  // print (data.container_width)
  // string sssw = to_string($width)
  // print("width")
  // myprint (sssw)

  int space = data.space

  int fixed_width = 0
  int nb_fixed_width = 0
  for item : data.items {
    // string sss = to_string($item.preferred_width)
    // myprint("pw " + sss)
    if ($item.preferred_width != -1) {
      fixed_width += $item.preferred_width
      nb_fixed_width++
    }
  }

  int space_per_item = 0
  if (nb_items != nb_fixed_width) {
    space_per_item = (width - fixed_width - (nb_items-1)*space - 2*padding_left) / (nb_items - nb_fixed_width)
    // string sss = to_string($space_per_item)
    // print("space_per_item ")
    // myprint (sss)
  }

  int dx = padding_left
  int max_height = 0
  for item : data.items {
    if ($item.preferred_width != -1) {
      item.width = $item.preferred_width
    } else {
      if ($item.max_width != -1) {
        // print(" " + item.max_width + " " + item.min_width + " ")
        item.width = space_per_item < $item.max_width ? (space_per_item < $item.min_width ? $item.min_width : space_per_item) : $item.max_width
      } else {
        item.width = space_per_item < $item.min_width ? $item.min_width : space_per_item
      }
      // print("item.width " + item.width)
    }

    item.x = dx
    dx += item.width + space
    // string sss = to_string($dx)
    // print("dx ")
    // myprint(sss)

    if ($item.preferred_height != -1) {
      item.height = $item.preferred_height
    } else {
      item.height = height - 2*padding_top
      if ($item.height > $item.max_height && $item.max_height > -1) {
        item.height = $item.max_height
      } else if ($item.height < $item.min_height) {
        item.height = $item.min_height
      }
    }
    if ($item.height > $max_height) {
      max_height = $item.height
    }
  }
  if ($data.items.size>0) {
    dx -= space // remove last added space FIXME requires non empty items
  }

  for item : data.items {
    if (item.v_alignment == 0) {        // top
      item.y = 0
    } else if (item.v_alignment == 1) { // center
      item.y = max_height/2 - item.height/2
    } else {                            // bottom
      item.y = max_height - item.height
    }
  }

  if (data.set_pos == 1) {
    if ($data.h_alignment == 0) {
      data.off_x = 0
    } else if ($data.h_alignment == 1) {
      data.off_x = (data.container_width - dx) / 2 - padding_left
    } else {
      data.off_x = data.container_width - dx - padding_left
    }

    if ($data.v_alignment == 0) {
      data.off_y = 0
    } else if ($data.v_alignment == 1) {
      data.off_y = (data.container_height - max_height) / 2 - padding_top
    } else {
      data.off_y = data.container_height - max_height - padding_top
    }
  }

  if (data.top_level_box != 1) {
    data.preferred_width = dx + 2*padding_left
    // print("data.preferred_width " + data.preferred_width)
    data.preferred_height = max_height + 2*padding_top
    data.cell_width = space_per_item
    data.cell_height = max_height
  }
}


_define_
HBox () inherits AbstractBox ()
{
  //String width_str("width")
  //String height_str("height")

  NativeAction update_items_pos_and_geom (fn_update_items_pos_and_geom, this, 0)
  
  this.container_width -> update_items_pos_and_geom
  this.container_height -> update_items_pos_and_geom

  SumList sum_of_min_width (this.items, "min_width")
  sum_of_min_width.output -> update_items_pos_and_geom // force geometry recomputation at startup, needed for text
  sum_of_min_width.output + this.space*(this.items.size - 1) =:> this.min_width

  MaxList max_of_min_height (this.items, "min_height")
  max_of_min_height.output =:> this.min_height

  this.min_width  -> update_items_pos_and_geom
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
