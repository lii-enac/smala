use core
use exec_env
use base
use gui

_define_
MenuItem (Process menu, string item, int init_x, int init_y)
{
  Translation pos (init_x, init_y)
  FontSize _ (5, 14)
  FontFamily _ ("B612")
  FillColor _ (0, 0, 0)
  Text text_item (2, 15.6, item)
  label aka text_item.text
  FSM selected {
    State st_unselected {
      NoFill _()
      NoOutline _()
      Rectangle r (0, 0, 0, 0, 0, 0)
      menu.width =: r.width
      menu.height =: r.height
    }
    State st_selected {
      FillColor _ (19, 82, 199)
      FillOpacity _ (0.6)
      Rectangle r (0, 0, 0, 0, 0, 0)
      menu.width =: r.width
      menu.height =: r.height
      AssignmentSequence set_sel (1) {
        label =: menu.selected
      }
      r.press->set_sel
    }
    st_unselected->st_selected (st_unselected.r.enter)
    st_selected->st_unselected (st_selected.r.leave)
    st_selected->st_unselected (st_selected.r.release, menu.end_selection)
  }
}