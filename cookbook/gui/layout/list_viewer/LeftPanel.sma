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
  
  Int width (200)
  Int height (100)

  // Properties
  String cur_flight_name_value ("")
  Int cur_flight_speed_value (0)
  Int cur_flight_level_value (0)

  // Vertical separation
  OutlineColor _ (200, 200, 200)
  OutlineWidth border_width (3)
  Line border (200, 0, 200, 500)
  OutlineWidth _ (1)

  Component bg1 {
    FillColor _ (#000088)
    FillOpacity _ (0.3)
    Rectangle r (0, 0, 0, 0, 15, 15)
    width =:> r.width
    height =:> r.height
  }


  ZOrderedGroup left_panel {
    RectangleClip clip (0, 0, 0, 0)

    Component bg2 {
      FillColor _ (#008800)
      FillOpacity _ (0.3)
      Rectangle r (0, 0, 0, 0, 5, 5)
    }

    FSM fsm {
      State idle {
        4 =: border_width.width
      }
      State hover {
        6 =: border_width.width
      }
      State moving {
        Double offset (0)
        f.press.x - border.x1 =: offset
        f.move.x - offset =:> border.x1, border.x2
      }
      idle->hover (border.enter)
      hover->idle (border.leave)
      hover->moving (border.press)
      moving->idle (border.release)
    }

    VBox vbox {
      vbox.h_alignment = 0
      vbox.space = 20

      HBox hbox1 {
        hbox1.space = 20

        Label label_flight_name ("Flight name")
        label_flight_name.preferred_width = 70
        UITextField flight_name
        // flight_name.preferred_width = 70
      }

      HBox hbox2 {
        hbox2.space = 20

        Label label_flight_level ("Flight level")
        label_flight_level.preferred_width = 70
        UITextField flight_level
        // flight_level.preferred_width = 70
      }
      
      HBox hbox3 {
        hbox3.space = 20

        Label label_flight_speed ("Flight speed")
        label_flight_speed.preferred_width = 70
        UITextField flight_speed
        // flight_speed.preferred_width = 70
      }
      
      PushButton btn_add ("Add flight")
      btn_add.h_alignment = 0
    }
  }
  // make widget naming independent from layout hierarchy
  btn_add aka left_panel.vbox.btn_add
  flight_name aka left_panel.vbox.hbox1.flight_name
  flight_level aka left_panel.vbox.hbox2.flight_level
  flight_speed aka left_panel.vbox.hbox3.flight_speed

  f.height =:> height, border.y2, left_panel.clip.height
  border.x1 =:> width, left_panel.clip.width
  
  // FIXME: sizes & positions does NOT match perfectly
  // left_panel.width =:> left_panel.bg2.r.width
  // left_panel.height =:> left_panel.bg2.r.height
  // left_panel.vbox.width =:> left_panel.bg2.r.width
  // left_panel.vbox.height =:> left_panel.bg2.r.height
  left_panel.vbox.min_width =:> left_panel.bg2.r.width
  left_panel.vbox.min_height =:> left_panel.bg2.r.height
  left_panel.vbox.off_x =:> left_panel.bg2.r.x
  left_panel.vbox.off_y =:> left_panel.bg2.r.y
  // left_panel.vbox.off_x + left_panel.vbox.padding_left =:> left_panel.bg2.r.x
  // left_panel.vbox.off_y + left_panel.vbox.padding_top =:> left_panel.bg2.r.y


  btn_add.click-> create_flight:(this) {
    // Add a new model of flight in the list
    addChildrenTo this.flight_list {
      // Create a new model of flight
      Component _ {
        String flight (getString (this.cur_flight_name_value))
        Int speed ($this.cur_flight_speed_value)
        Int FL ($this.cur_flight_level_value)
      }
    }
  }

  // Key "tab" allows to set the focus to next text field
  flight_name.next -> flight_level.activate
  flight_level.next -> flight_speed.activate
  flight_speed.next -> btn_add.select

  flight_name.text =:> cur_flight_name_value
  flight_level.text =:> cur_flight_level_value
  flight_speed.text =:> cur_flight_speed_value

  create_flight->flight_name.clear, flight_level.clear, flight_speed.clear


  // for DEBUG purpose
  TextPrinter tp
  TextPrinter tp2
  
  // flight_name.text + " - " + flight_level.text + " - " + flight_speed.text =:> tp.input
  // "left_panel.vbox w/h = " + left_panel.vbox.min_width + " x " + left_panel.vbox.min_height =:> tp2.input
  // "left_panel.vbox padding left/top = " + left_panel.vbox.padding_left + " x " + left_panel.vbox.padding_top =:> tp2.input

}
