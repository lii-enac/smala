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

import core.ontology.process
import core.tree.container
import base.process_handler


_native_code_
%{
#include "core/utils/iostream.h"

//using string = djnnstl::string;

// static Process* find_without_warning (Process* p, string path)
// {
//   if (p == nullptr)
//     return 0;
//   return p->find_child_impl(path);
// }

static void
check (Process* p, const djnnstl::string& p_name)
{
	if (find_optional (p, p_name) == 0) {
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
  int preferred_width = data.preferred_width
  int preferred_height = data.preferred_height
  int width = data.preferred_width == -1 ? data.container_width - 2*padding_left : data.preferred_width - 2*padding_left
  int height = data.preferred_height == -1 ? data.container_height - 2*padding_top : data.preferred_height - 2*padding_top

  int space = data.space

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
    space_per_item = (width - fixed_width - (nb_items-1)*space - 2*padding_left) / (nb_items - nb_fixed_width)
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
    dx += item.width + space

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
  dx -= space
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
  data.preferred_width = dx + 2*padding_left
  data.preferred_height = max_height + 2*padding_top
  data.cell_width = space_per_item
  data.cell_height = max_height
}

_define_
HBox (Process container) inherits IWidget ()
{
  check (container, "width")
  check (container, "height")
  Bool set_pos (1)
  Translation offset (0, 0)
  off_x aka offset.tx
  off_y aka offset.ty
  Translation padding (5, 5)
  padding_left aka padding.tx
  padding_top aka padding.ty
  Int cell_width (0)
  Int cell_height (0)
  Int space (5)
  if (find_optional (container, "cell_width")) {
    container_width aka container.cell_width
    container_height aka container.cell_height
    set_pos = 0
    padding_left = 0
    padding_top = 0
  } else {
    container_width aka container.width
    container_height aka container.height
  }

  ZOrderedGroup g {
    List items
  }
  items aka g.items
  
  NativeAction update_items_pos_and_geom (fn_update_items_pos_and_geom, this, 0)
  this.container_width->update_items_pos_and_geom
  this.container_height->update_items_pos_and_geom 
  SumList sl (items, "min_width")
  MaxList ml (items, "min_height")
  sl.output->update_items_pos_and_geom // bad trick to force geometry recomputation at startup, needed for text
  sl.output + space*(items.size - 1) =:> this.min_width
  ml.output =:> this.min_height
}
