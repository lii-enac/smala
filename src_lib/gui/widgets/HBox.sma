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


_action_
fn_update_items_pos_and_geom (Process src, Process data)
{
  int is_sub_box = data.is_sub_box
  int nb_items = data.items.size
  int width = data.width
  int height = data.height
  int hspace = data.hspace
  int vspace = data.vspace
  int item_width = is_sub_box ? (width - (nb_items - 1)*hspace)/nb_items : (width - (nb_items + 1)*hspace)/nb_items
  int item_height = is_sub_box ? height : height - 2*vspace
  int dx = is_sub_box ? 0 : hspace
  for item : data.items {
    item.x = dx
    item.y = is_sub_box ? 0 : vspace
    item.req_width = item_width
    item.req_height = item_height
    dx = dx + hspace + item_width
  }
}

_action_
fn_set_min_size (Process src, Process data)
{
  int is_sub_box = data.is_sub_box
  int nb_items = data.items.size
  int hspace = data.hspace
  int vspace = data.vspace
  int dx = hspace
  int min_width = is_sub_box ? 0 :  2 * hspace
  int min_height = is_sub_box ? 0 : 2 * vspace
  for item : data.items {
    if (item.min_width > min_width) {
      min_width = item.min_width
    }
    if (item.min_height > min_height) {
      min_height = is_sub_box ? item.min_height : 2 * vspace + item.min_height
    }
    p = find (item, "change_parent")
    if (&p != 0) {
      run item.change_parent
    }
  }
  data.min_height = min_height
  if (is_sub_box) {
    data.min_width = min_width * nb_items + hspace * (nb_items - 1)
  } else {
    data.min_width = min_width * nb_items + hspace * (nb_items + 1)
  }
}

_define_
HBox (int _is_sub_box)
{
  parent = find (this, "..")
  width aka parent.width
  height aka parent.height
  Int is_sub_box (_is_sub_box)
  Int hspace (5)
  Int vspace (5)
  Int min_width (0)
  Int min_height (0)
  List items
  List hover
  NativeAction update_items_pos_and_geom (fn_update_items_pos_and_geom, this, 0)
  width->update_items_pos_and_geom
  NativeAction set_min_size (fn_set_min_size, this, 0)
  min_width =:> parent.min_width
  min_height =:> parent.min_height
}