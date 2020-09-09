use core
use base
use display
use gui

import Slider


_define_
Tab (Process frame, Process tabManager, string _label, int _index) {
  /* -------- Graphic components --------- */
  FillColor foreground_color (250, 250, 250)
  g_tab = loadFromXML ("img/tab.svg")

  Component label_box {
    Translation pos ((_index - 1) * 160, 0)
    tab << g_tab.layer1.tab
    RectangleClip clip (0, 0, 150, 25)
    FillColor font_color (0, 0, 0)
    FontSize fs (0, 11)
    Text label (25, 22, _label)
  }
  label aka label_box.label.text

  Component pane {
    FillOpacity opacity (1)
  }

  Component border {
    OutlineWidth ow (1)
    OutlineColor border_color (100, 100, 100)
    Line left (0, 22, 50, 22)
    Line right (50, 22, 300, 22)
    label_box.pos.tx =: left.x2
    g_tab.layer1.left_pos.y =: left.y1, left.y2, right.y1, right.y2
    label_box.pos.tx + g_tab.layer1.right_pos.x =: right.x1
    frame.width =: right.x2
  }
  border_color aka border.border_color
  /* ------ End graphic components ------- */


  /* ----- Interface with TabManager ----- */
  Int index (_index)
  Spike select
  Spike unselect
  AssignmentSequence notify_selected (1) {
    index =: tabManager.selected_index
  }
  /* --- End interface with TabManager --- */


  FSM fsm {
    State st_unselected {
      tabManager.unselected_color =: foreground_color.r, foreground_color.g, foreground_color.b, label_box.tab.fill.r, label_box.tab.fill.g, label_box.tab.fill.b
      tabManager.unselected_font_color =: label_box.font_color.r, label_box.font_color.g, label_box.font_color.b
      tabManager.unselected_border_color =: label_box.tab.stroke.r, label_box.tab.stroke.g, label_box.tab.stroke.b
    }

    State st_selected {
      tabManager.selected_color =: foreground_color.r, foreground_color.g, foreground_color.b,label_box.tab.fill.r, label_box.tab.fill.g, label_box.tab.fill.b
      tabManager.selected_font_color =: label_box.font_color.r, label_box.font_color.g, label_box.font_color.b
      tabManager.selected_border_color =: border_color.r, border_color.g, border_color.b, label_box.tab.stroke.r, label_box.tab.stroke.g, label_box.tab.stroke.b

      Slider s (frame, 300, 50)
      frame.width - s.width - 30 =:> s.x
      frame.height - 30 =:> s.y
      s.output =:> pane.opacity.a
    }

    st_unselected -> st_selected   (label_box.tab.press, notify_selected)
    st_unselected -> st_selected   (select, notify_selected)
    st_selected   -> st_unselected (unselect)
  }
}