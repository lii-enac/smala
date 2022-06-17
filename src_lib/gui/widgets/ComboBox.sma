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
fn_init_items_pos_and_geom (Process src, Process data)
{
  addChildrenTo data.fsm.st_dpy.text_items {
    int dy = 0
    for item : data.str_items {
      ComboBoxItem _ (data, item, 0, dy)
      dy = dy + 15
    }
  }
}

_define_
ComboBox () inherits IWidget () {
  mouseTracking = 1
  Incr incr (1)
  Incr decr (1)
  Bool is_hover (0)
  incr.state - decr.state != 0 =:> is_hover

  /*----- interface -----*/
  Spike post_init
  Spike unselect

  /*----- interface -----*/

  Int idle_color (#323232)
  Int pressed_color (#959595)
  Int text_color (#ffffff)
  OutlineColor _ (#535353)
  FillColor fc (#535353)
  Rectangle r (0, 0, 100, 20, 3, 3)
  
  Component arrow {
    FillColor _ (Black)
    Translation pos (0, 10)
    Polygon p {
      Point _ (0, 0)
      Point _ (12, 0)
      Point _ (6, 12)
    }
    r.width - 16 =:> pos.tx
    (r.height - 12)/2.0 + 1 =:> pos.ty
  }
  Component selected_item {
    RectangleClip clip (0, 0, 0, 40)
    FillColor _ (White)
    Text t (5, 0, "")
  }
  selected_item.t.height + 10 =:> this.min_height
  this.min_height = selected_item.t.height + 10
  this.preferred_height = selected_item.t.height + 10

  r.height/2.0 + (selected_item.t.ascent - selected_item.t.descent)/2.0 - 1 =:> selected_item.t.y

  this.width - 20 =:> selected_item.clip.width

  value aka selected_item.t.text

  this.min_height = 20
  this.preferred_height = 20

  r.height =:> this.min_height, this.preferred_height

  List str_items

  FSM fsm {
    State st_idle
    State st_dpy {
      0 =: incr.state, decr.state
      FillColor _ (White)
      OutlineColor _ (#323232)
      Rectangle bg (0, 0,100, 70, 0, 0)
      RectangleClip clip (0, 0, 100, 70)
      bg.{width,height} => clip.{width, height}

      r.width =:> bg.width
      r.height =:> bg.y, clip.y
      bg.z = 10

      Translation t (0, 15)
      r.height + 15 =:> t.ty
      List text_items

      FSM check_hover {
        State not_hover {
          GenericMouse.left.press -> unselect
        }
        State hover
        hover->not_hover (is_hover.false)
        not_hover->hover (is_hover.true)
      }
    }
    st_idle->st_dpy (r.press)
    st_dpy->st_idle (r.press)
    st_dpy->st_idle (unselect)
  }
  max_content_height aka fsm.st_dpy.bg.height
  NativeAction init_items_pos_and_geom (fn_init_items_pos_and_geom, this, 0)

  str_items.$added -> (this) {
    int size = getInt (this.fsm.st_dpy.text_items.size)
    int y = (size + 1) * 15
    for item : this.str_items {
      if (has_item (item, this.fsm.st_dpy.text_items) == 0) {
        addChildrenTo this.fsm.st_dpy.text_items {
          ComboBoxItem new_item (this, item, 0, y)
          y += 15
        }
      }
    }
  }
  str_items.$removed -> (this) {
    for item : this.fsm.st_dpy.text_items {
      if (has_str_item (item, this.str_items) == 0) {
          delete item
      }
    }
    int y = 15
    for item : this.fsm.st_dpy.text_items {
      item.y = y
      y += 15
    }
  }
}
