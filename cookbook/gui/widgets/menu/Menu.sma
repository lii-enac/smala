use core
use exec_env
use base
use gui

import MenuItem

_define_
Menu (Process f, Process list_item, int init_x, int init_y, int _width)
{
  /* interface */
  Translation pos (init_x, init_y)
  x aka pos.tx
  y aka pos.ty

  Spike end_selection
  String selected ("")

  /* graphics */
  graphics = loadFromXML ("img/menu.svg")
  bkg << graphics.layer1.bkg
  Component selected_item {
   RectangleClip clip (0, 0, 0, 0)
   label << graphics.layer1.item
  }
  selected =:> selected_item.label.text
  Component selector {
    Translation pos_sel (0, 0)
    selector << graphics.layer1.selector
  }
  mask_selector << graphics.layer1.mask_selector
  list_item.1 =: selected_item.label.text, selected


  /* geometry management */
  Int width (_width)
  Int height (0)
  bkg.height =:> height, selected_item.clip.height
  width =:> bkg.width
  width - mask_selector.width - 2 =:> selected_item.clip.width
  width - mask_selector.width =:> mask_selector.x
  mask_selector.x + 2.63 =:> selector.pos_sel.tx

  Switch sw_panel (hidden) {
    Component hidden
    Component visible {
      Translation loc_pos (0, 0)
      FillColor _ (255, 255, 255)
      OutlineColor _ (188, 188, 188)
      Rectangle sel_bkg (0, 0, 100, 100, 0, 0)
      RectangleClip clip (0, 0, 0, 0)
      width =:> sel_bkg.width, clip.width

      FillColor _ (0, 0, 0)
      bkg.height =: loc_pos.ty
      List items {
        int y = 0
        for (int i = 1; i <= $list_item.size; i++) {
          MenuItem nt (this, "", 0, y)
          nt.label = toString (list_item.[i])
          y = y + bkg.height
        }
      }
      items.size * bkg.height =:> sel_bkg.height, clip.height
    }
  }
  FSM fsm {
    State hidden
    State visible
    hidden->visible (mask_selector.press)
    visible->hidden (end_selection)
    visible->hidden (mask_selector.press)
  }
  fsm.state =:> sw_panel.state
}