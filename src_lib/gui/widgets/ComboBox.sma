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

_native_code_
%{
Process* getParent (Process *p)
{
  return p->get_parent ();
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
fn_update_items_pos_and_geom (Process src, Process data)
{
  
  addChildrenTo data.for_hover.fsm.st_dpy {
    List text_items { 
      int dy = 15
      for item : data.str_items {
        Component _ {
          FillColor fc (White)
          NoOutline _
          Rectangle r (0, dy - 13, 0, 16, 0, 0)
          FillColor _ (#323232)
          Text t (3, dy, toString (item))
          data.r_selected.width - 20 =:> r.width
          dy = dy + 15
          FSM behavior {
            State st_idle {
              #FFFFFF =: fc.value
            }
            State st_hover {
              #999999 =: fc.value
              r.press-> { t.text =: data.selected_item.text}
              data.selected_item.text->data.unselect
            }
            st_idle->st_hover (r.enter)
            st_hover->st_idle (r.leave)
          }
        }
      }
    }
    text_items.size * 15 + 3 =:> data.for_hover.fsm.st_dpy.bg.height
    MaxList max (text_items, "t/width")
    max.output =:> data.min_width
  }
}

_define_
ComboBox (Process container, double x_, double y_) inherits IWidget (container) {
  mouseTracking = 1
  Translation t (x_, y_)
  
  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  Spike post_init
  Spike unselect
  /*----- interface -----*/

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
  r_selected.width - 20 =:> r_selection.x, arrow.pos.tx

  //selected_item.width + 25 =:> this.min_width
  this.min_height = 20
  this.width  =:> r_selected.width
  this.height/2 - 10 =:> offset.ty

  List str_items
  Component for_hover {
    FSM fsm {
      State st_idle {
        0 =: arrow.r.a, arrow.pos2.ty
      }
      State st_dpy {
        -90 =: arrow.r.a
        2 =: arrow.pos2.ty
        FillColor _ (White)
        OutlineColor _ (#323232)
        Rectangle bg (0, 0,100, 100, 0, 0)
        r_selected.width - 20 =:> bg.width
        FillColor _ (#323232)
        
      }
      st_idle->st_dpy (r_selection.press)
      st_dpy->st_idle (r_selection.press)
      st_dpy->st_idle (unselect)
    }
  }
  NativeAction update_items_pos_and_geom (fn_update_items_pos_and_geom, this, 0)
  NativeAction change_parent (fn_change_parent, this, 1)
}
