use core
use base
use gui


import gui.widgets.PushButton
import gui.widgets.HBox
import gui.widgets.VBox
import gui.widgets.Label
import gui.widgets.HSpace
import gui.widgets.UITextField
import gui.widgets.ListViewer

_define_
LeftPanel (Process f, Process model)
{
  flight_list aka model
  Component left_panel {
    Int width (200)
    Int height (100)
    OutlineColor _(200, 200, 200)
    OutlineWidth _ (4)
    Line border (200, 0, 200, 500)
    f.height =:> border.y2, height
    border.x1 =:> width
  }
  width aka left_panel.width
  height aka left_panel.height
  f.height =:> left_panel.height
  String cur_flight_name_value ("")
  Int cur_flight_speed_value (0)
  Int cur_flight_level_value (0)

  VBox vbox (left_panel)
    vbox.space = 10
    HBox hbox1 (vbox)
      hbox1.space = 20
      Label label_flight_name ("Flight name")
      label_flight_name.preferred_width = 70
      UITextField flight_name
      flight_name.preferred_width = 70

    HBox hbox2 (vbox)
      hbox2.space = 20
      Label label_flight_level ("Flight level")
      label_flight_level.preferred_width = 70
      UITextField flight_level
      flight_level.preferred_width = 70
    
    HBox hbox3 (vbox)
      hbox3.space = 20
      Label label_flight_speed ("Flight speed")
      label_flight_speed.preferred_width = 70
      UITextField flight_speed
      flight_speed.preferred_width = 70

  flight_name.next->flight_level.activate
  flight_level.next->flight_speed.activate

  flight_name.text =:> cur_flight_name_value
  flight_level.text =:> cur_flight_level_value
  flight_speed.text =:> cur_flight_speed_value
  PushButton add ("Add flight")
  add.h_alignment = 0
  add.click-> create_flight:(this) {
    addChildrenTo this.flight_list {
      Component _ {
        String flight (getString (this.cur_flight_name_value))
        Int speed ($this.cur_flight_speed_value)
        Int FL ($this.cur_flight_level_value)
      }
    }
  }
  create_flight->flight_name.clear, flight_level.clear, flight_speed.clear

  addChildrenTo hbox1.items {
    label_flight_name, flight_name
  }
  addChildrenTo hbox2.items {
    label_flight_level, flight_level
  }
  addChildrenTo hbox3.items {
    label_flight_speed, flight_speed
  }
  addChildrenTo vbox.items {
    hbox1,
    hbox2,
    hbox3,
    add
  }
}