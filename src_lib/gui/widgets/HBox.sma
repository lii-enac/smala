/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2021)
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

import gui.widgets.IWidget

_native_code_
%{
#include <iostream>
static Process* find_without_warning (Process* p, string path)
{
  if (p == nullptr)
    return 0;
  return p->find_child_impl(path);
}

static void
check (Process* p, const string& p_name)
{
	if (find_without_warning (p, p_name) == 0) {
		std::cerr << "Process " << p_name << " not found in VBox initialisation\n";
		exit (0);
	}
}

%}

_action_
fn_update_items_pos_and_geom (Process src, Process data)
{
  int nb_items = data.items.size

  int padding_left = data.padding_left
  int padding_top = data.padding_top
  int width = data.container_width - 2*padding_left
  int height = data.container_height - 2*padding_top

  int hspace = data.hspace

  int fixed_width = 0
  int nb_fixed_width = 0
  for item : data.items {
    if ($item.preferred_width != -1) {
      fixed_width += $item.preferred_width
      nb_fixed_width++
    }
  }
  int space_per_item = 0
  if (nb_items != nb_fixed_width) {
    space_per_item = (width - fixed_width - (nb_items-1)*hspace - 2*padding_left) / (nb_items - nb_fixed_width)
  }
  int dx = 0
  int max_height = 0
  for item : data.items {
    if ($item.preferred_width != -1) {
      item.width = $item.preferred_width
    } else {
      if ($item.max_width != -1) {
        item.width = space_per_item < $item.max_width ? (space_per_item < $item.min_width ? $item.min_width : space_per_item) : $item.max_width
      } else {
        item.width = space_per_item < $item.min_width ? $item.min_width : space_per_item
      }
    }
    item.x = dx
    dx += item.width + hspace

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
      data.x = padding_left
    } else if ($data.h_alignment == 1) {
      data.x = (data.container_width - dx) / 2
    } else {
      data.x = data.container_width - dx - padding_left
    }

    if ($data.v_alignment == 0) {
      data.y = padding_top
    } else if ($data.v_alignment == 1) {
      data.y = (data.container_height - max_height) / 2
    } else {
      data.y = data.container_height - max_height - padding_top
    }
  }
  data.width = dx + 2*padding_left
  data.height = max_height + 2*padding_top
  data.cell_width = space_per_item
  data.cell_height = max_height
}

_define_
HBox (Process container) inherits IWidget ()
{
  check (container, "width")
  check (container, "height")
  Bool set_pos (1)
  Int padding_left (5)
  Int padding_top (5)
  Int cell_width (0)
  Int cell_height (0)
  Int hspace (5)
  if (find_without_warning (container, "cell_width")) {
    container_width aka container.cell_width
    container_height aka container.cell_height
    set_pos = 0
    padding_left = 0
    padding_top = 0
  } else {
    container_width aka container.width
    container_height aka container.height
  }

  List items
  List hover
  NativeAction update_items_pos_and_geom (fn_update_items_pos_and_geom, this, 0)
  this.container_width->update_items_pos_and_geom
  this.container_height->update_items_pos_and_geom 
  SumList sl (items, "min_width")
  MaxList ml (items, "min_height")
  sl.output->update_items_pos_and_geom // bad trick to force geometry recomputation at startup, needed for text
  sl.output + hspace*(items.size - 1) =:> this.min_width
  ml.output =:> this.min_height
}
