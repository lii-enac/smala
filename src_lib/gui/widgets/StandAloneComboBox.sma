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
use animation

import gui.animation.Animator
import gui.widgets.IWidget
import ComboBoxItem

_native_code_
%{
static Process* getParent (Process *p)
{
  return p->get_parent ();
}

static int has_item (Process *item, Process *list) {
  string str_item = ((djnn::TextProperty*)item)->get_value ();
  for (auto p : ((djnn::List*)list)->children ()) {
    if (((djnn::TextProperty*)item)->get_value () == ((djnn::TextProperty*)p->find_child("t/text"))->get_value ()) {
      return 1;
    }
  }
   return 0;
}

static int has_str_item (Process *item, Process *list) {
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
  if (!data.local_panel_in_upper_layer)
  {
    data.local_panel_in_upper_layer = 1
    upper_layer = getRef (data.upper_layer)
    addChildrenTo upper_layer {
      data.local_panel
    }
  }

}

_action_
fn_init_items_pos_and_geom (Process src, Process data)
{
  addChildrenTo data.text_items {
    int dy = 65
    for item : data.str_items {
      ComboBoxItem _ (data, item, 25, dy)
      dy = dy + data.szItem
    }
  }
}

_define_
StandAloneComboBox (Process _upper_layer, string _default, double x_, double y_) {


  /*----- interface -----*/
  Double x (x_)
  Double y (y_)

  Spike post_init
  Spike unselect
  Spike update
  Int nbItems (0)
  Int szItem (17)
  Int max_height (120)
  /*----- interface -----*/
  Bool local_panel_in_upper_layer (0)
  RefProperty upper_layer (_upper_layer)
  String value (_default)
  selected_item aka value
  Int default_width (200)
  Incr incr (1)
  Incr decr (1)
  Bool is_hover (0)
  incr.state - decr.state != 0 =:> is_hover

  /*----- graphics -----*/
  Component ui {
    Component bg {
      FillColor _ (#303030)
      Rectangle bg (0, 0, 196, 50, 5, 5)
      width aka bg.width
      height aka bg.height
    }
    Component main_box {
      FillColor _ (#ffffff)
      OutlineColor _ (#303030)
      Rectangle main_box (24, 10, 148, 30, 5, 5)
      width aka main_box.width
      height aka main_box.height
    }
    Component bg_arrow {
      FillColor _ (#656565)
      FillOpacity fo (0.14)
      alpha aka fo.a
      NoOutline _
      Path bg_arrow ("m 169.02539,39.556641 c 0.56445,-0.250509 1.08339,-0.5873 1.51367,\
      -1.017579 0.40638,-0.40638 0.71379,-0.902493 0.96094,-1.429687 V 12.734996 c -0.24715,\
      -0.527194 -0.55456,-1.023307 -0.96094,-1.429687 -0.39807,-0.398075 -0.88601,-0.695739 -1.40039,-0.941407 h -14.30078 v 29.192739 z")
      enter aka bg_arrow.enter
      leave aka bg_arrow.leave
      press aka bg_arrow.press
      release aka bg_arrow.release
    }
    Component arrow {
      FillColor _ (#ffa700)
      OutlineColor _ (#ffa700)
      OutlineWidth _ (1.77)
      OutlineCapStyle _(2)
      OutlineJoinStyle _(1)
      OutlineMiterLimit _ (4)
      Path arrow ("m 157.66126,20.571361 5.59732,8.85735 5.59915,-8.85735 z")
    }
    Component text {
      FontSize _ (5, 14)
      FontFamily _ ("B612")
      FillColor _ (#303030)
      Text t (30.3, 30.6, _default)
      width aka t.width
      height aka t.height
      text aka t.text
    }
    Component panel {
      FillColor _ (#ffffff)
      OutlineColor _ (#303030)
      OutlineWidth _(2)
      Rectangle panel (24, 42, 148, 120, 5, 5)
      width aka panel.width
      height aka panel.height
      x aka panel.x
      y aka panel.y
      enter aka panel.enter
      leave aka panel.leave
    }
  }

  Component main_label {
    Translation t (x_, y_)
    x =:> t.tx
    y =:> t.ty
    bg << ui.bg
    main_box << ui.main_box
    main_box.width + 48 =:> bg.width
    text << ui.text
    Component right_side {
      Translation pos_arrow (0, 0)
      main_box.width - 148 =:> pos_arrow.tx
      bg_arrow << ui.bg_arrow
      Animator anim (200, 0, 1, DJN_OUT_SINE, 0, 0)
      FSM anim_op {
        State hidden {
          0 =: bg_arrow.alpha
        }
        State to_visible {
          anim.output => bg_arrow.alpha
        }
        State visible {
          1 =: bg_arrow.alpha
        }
        State to_hidden {
          anim.output => bg_arrow.alpha
        }
        hidden->to_visible (bg_arrow.enter, anim.start)
        to_visible->visible  (anim.end)
        {to_visible, visible}->to_hidden (bg_arrow.leave, anim.rewind)
        to_hidden->hidden (anim.end)
      }
      arrow << ui.arrow
    }
  }
  width aka main_label.main_box.width
  text aka main_label.text.text
  //text =:> value
  value =:> text
  Bool display_lift (0)
  

  List str_items
  if (getString(value) != "") {
    addChildrenTo str_items {
      String _(_default)
    }
  }
  Component local_panel {
    Translation t (x_, y_)
    x =:> t.tx
    y =:> t.ty
    

    FSM fsm_panel {
      State idle
      State visible {
        0 =: incr.state, decr.state
        panel << ui.panel
        Component clipped {
          RectangleClip clip (0, 0, 0, 0)
          panel.{x,y,width} =:> clip.{x,y,width}
          panel.height - 2 =:> clip.height
          FontSize _ (5, 14)
          FontFamily _ ("B612")
          FillColor _ (Black)
          Translation lift (0, 0)
          BoundedValue bv_lift (0, 0, 0)
          List text_items
          text_items.size =:> nbItems
          nbItems * szItem < max_height ? nbItems*szItem + 10 : max_height =:> panel.height
          -(nbItems-1) * szItem =:> bv_lift.min
          GenericMouse.wheel.dy > 0 -> { lift.ty + szItem =: bv_lift.input}
          GenericMouse.wheel.dy < 0 -> { lift.ty - szItem =: bv_lift.input}
          bv_lift.result =:> lift.ty
        }
        main_label.main_box.width =:> panel.width
        FSM check_hover {
          State not_hover {
            GenericMouse.left.press -> unselect
          }
          State hover
          hover->not_hover (is_hover.false)
          not_hover->hover (is_hover.true)
        }
      }
      idle->visible (main_label.right_side.bg_arrow.press)
      visible->idle (main_label.right_side.bg_arrow.press)
      visible->idle (unselect)
    }
  }
  text_items aka local_panel.fsm_panel.visible.clipped.text_items

  str_items.$added -> (this) {
    int size = getInt (this.text_items.size)
    int y = (size + 1) * this.szItem + 42
    for item : this.str_items {
      if (has_item (item, this.text_items) == 0) {
        addChildrenTo this.text_items {
          ComboBoxItem new_item (this, item, 25, y)
          y += this.szItem
        }
      }
    }
  }
  update->(this) {
    delete_content this.text_items
    int y = 65
    for item : this.str_items {
      addChildrenTo this.text_items {
        ComboBoxItem new_item (this, item, 25, y)
        y += this.szItem
      }
    }
  }
  
  str_items.$removed -> (this) {
    for item : this.text_items {
      if (has_str_item (item, this.str_items) == 0) {
        delete item
      }
    }
    int y = 65
    for item : this.text_items {
      item.y = y
      y += this.szItem
    }
  }
  NativeAction init_items_pos_and_geom (fn_init_items_pos_and_geom, this, 0)
  MaxList max (text_items, "t/width")
  str_items.size == 0 ? default_width : max.output + 50 =: main_label.main_box.width
  max.output + 50 => main_label.main_box.width
  NativeAction change_parent (fn_change_parent, this, 0)
}
