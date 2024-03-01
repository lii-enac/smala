use core
use base
use gui

_define_
ComboBoxItem (Process box, Process item, int dx, int dy) {
  Translation pos (dx, dy)
  y aka pos.ty
  ZOrder z (11)
  Component bg {
    //FillOpacity _ (0.5)
    FillColor fc (White)
    NoOutline _
    Rectangle r (2, - 13, 0, 17, 2, 2)
  }
  //FillColor _ (#8fa4af)
  FillColor tc (#303030)
  Text t (4, 0, toString (item))
  box.width - 6 =:> bg.r.width
  FSM behavior {
    State st_idle {
      #FFFFFF =: bg.fc.value
      #303030 =: tc.value
    }
    State st_hover {
      //#8fa4af =: bg.fc.value
      #197EFF =: bg.fc.value
      #FFFFFF =: tc.value
      bg.r.press-> { t.text =: box.value}
      box.value->box.unselect
    }
    st_idle->st_hover (bg.r.enter, box.incr)
    st_hover->st_idle (bg.r.leave, box.decr)
  }
  behavior.st_hover->(this) {
    moveChild this >>
  }
}