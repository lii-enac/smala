use core
use base
use gui

_define_
ComboBoxItem (Process box, Process item, int dy) {
  FillColor fc (White)
  NoOutline _
  Translation pos (0, dy)
  y aka pos.ty
  Rectangle r (0, - 13, 0, 16, 0, 0)
  FillColor _ (#323232)
  Text t (3, 0, toString (item))
  box.r_selected.width - 20 =:> r.width
  FSM behavior {
    State st_idle {
      #FFFFFF =: fc.value
    }
    State st_hover {
      #999999 =: fc.value
      r.press-> { t.text =: box.selected_item.text}
      box.selected_item.text->box.unselect
    }
    st_idle->st_hover (r.enter)
    st_hover->st_idle (r.leave)
  }
}