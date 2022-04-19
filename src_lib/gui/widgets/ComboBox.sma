/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
use core
use base
use display
use gui

import gui.widgets.IWidget
import ComboBoxItem

_native_code_
%{
Process* getParent (Process *p)
{
  return p->get_parent ();
}

int has_item (Process *item, Process *list) {
  string str_item = ((djnn::TextProperty*)item)->get_value ();
  for (auto p : ((djnn::List*)list)->children ()) {
    if (((djnn::TextProperty*)item)->get_value () == ((djnn::TextProperty*)p->find_child("t/text"))->get_value ()) {
      return 1;
    }
  }
   return 0;
}

int has_str_item (Process *item, Process *list) {
  string str_item = ((djnn::TextProperty*)item->find_child("t/text"))->get_value ();
  for (auto p : ((djnn::List*)list)->children ()) {
    if ( ((djnn::TextProperty*)p)->get_value () == str_item) {
      return 1;
    }
  }
   return 0;
}

%}
_action_
fn_change_parent (Process src, Process data)
{
  parent = getParent (data)
  gparent = getParent (parent)
  addChildrenTo gparent.hover {
    Translation pos (0, 0)
    data.t.ty + data.offset.ty + 20 =:> pos.ty
    data.t.tx + data.offset.tx =:> pos.tx
  }
  addChildrenTo gparent.hover {
    data.for_hover
  }
}

_action_
fn_init_items_pos_and_geom (Process src, Process data)
{
  addChildrenTo data.for_hover.fsm.st_dpy.text_items {
    int dy = 15
    for item : data.for_hover.str_items {
      ComboBoxItem _ (data, item, 0, dy)
      dy = dy + 15
    }
  }
}

_define_
ComboBox (Process container, double x_, double y_, int _default_width, int _width_by_content) inherits IWidget (container) {
  mouseTracking = 1
  Translation t (x_, y_)
  Int default_width (_default_width)

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  Spike post_init
  Spike unselect

  /*----- interface -----*/
  Incr incr (1)
  Incr decr (1)
  Bool is_hover (0)
  incr.state - decr.state != 0 =:> is_hover

  Translation offset (0, 0)
  FillColor fc (#ffffff)
  OutlineColor oc (#323232)
  Rectangle r_selected (0, 0, 200, 20, 0, 0)
  FillColor fc2 (#323232)
  Text selected_item (3, 15, "")
  Rectangle r_selection (180, 0, 20, 20, 0, 0)
  Component arrow {
    FillColor _ (White)
    Translation pos (0, 0)
    Translation pos2 (0, 0)
    Rotation r (0, 10, 10)
    Polygon p {
      Point _ (15,5)
      Point _ (5,10)
      Point _ (15, 15)
    }
  }
  width aka r_selected.width
  text aka selected_item.text
  r_selected.width - 20 =:> r_selection.x, arrow.pos.tx

  //selected_item.width + 25 =:> this.min_width
  this.min_height = 20
  
  this.height/2 - 10 =:> offset.ty


  Component for_hover {
    List str_items
    FSM fsm {
      State st_idle {
        0 =: arrow.r.a, arrow.pos2.ty
      }
      State st_dpy {
         0 =: incr.state, decr.state
        -90 =: arrow.r.a
        2 =: arrow.pos2.ty
        FillColor _ (White)
        OutlineColor _ (#323232)
        Rectangle bg (0, 0,100, 100, 0, 0)
        r_selected.width - 20 =:> bg.width
        FillColor _ (#323232)
        List text_items
        text_items.size * 15 + 3 =:> bg.height
        FSM check_hover {
          State not_hover {
            GenericMouse.left.press -> unselect
          }
          State hover
          hover->not_hover (is_hover.false)
          not_hover->hover (is_hover.true)
        }
      }
      st_idle->st_dpy (r_selection.press)
      st_dpy->st_idle (r_selection.press)
      st_dpy->st_idle (unselect)
    }
  }

  for_hover_fsm aka for_hover.fsm
  str_items aka for_hover.str_items
  MaxList max (for_hover.fsm.st_dpy.text_items, "t/width")
  if (_width_by_content) {
    for_hover.str_items.size  == 0 ? default_width : max.output + 50 =:> r_selected.width
  } else {
    this.width  =:> r_selected.width
  }

  NativeAction init_items_pos_and_geom (fn_init_items_pos_and_geom, this, 0)
  str_items.$added -> (this) {
    int size = getInt (this.for_hover_fsm.st_dpy.text_items.size)
    int y = (size + 1) * 15
    for item : this.str_items {
      if (has_item (item, this.for_hover_fsm.st_dpy.text_items) == 0) {
        addChildrenTo this.for_hover_fsm.st_dpy.text_items {
          ComboBoxItem new_item (this, item, 0, y)
          y += 15
        }
      }
    }
  }
  str_items.$removed -> (this) {
    for item : this.for_hover_fsm.st_dpy.text_items {
      if (has_str_item (item, this.str_items) == 0) {
          delete item
      }
    }
    int y = 15
    for item : this.for_hover_fsm.st_dpy.text_items {
      item.y = y
      y += 15
    }
  }

  NativeAction change_parent (fn_change_parent, this, 1)
}
