use core
use base
use gui

_define_
FittsTask1D(Component f) {
  Int target_width(10)
  Int target_distance(500)
  Int target_time_acquisition(0)

  // bg
  FillColor _(0,0,0)
  Rectangle bg(0,0, 0,0, 0,0)
  f.width => bg.width
  f.height => bg.height

  // scene
  Switch display(starting_area) {
    Component starting {
      FillOpacity o(1)
      FillColor fc(200, 200, 200) // lightgray
      Rectangle area (0,0, 32,32, 0,0)
      //(f.height-32)/2 => area.y
      f.height => area.height
    }
    Component target {
      FillColor fc(200, 200, 200) // lightgray
      Rectangle area (0,0, 0,0, 0,0)
      target_distance-target_width/2 => area.x
      target_width => area.width
      f.height => area.height
    }
  }

  // cursor
  FillColor _(255,255,255)
  OutlineColor _(0,0,0)
  OutlineWidth _(1)
  Rectangle cursor(0,0, 2,0, 0,0)
  f.height => cursor.height
  f.move.x => cursor.x

  Bool in(0)

  FSM control {
    State init {
      sfc aka display.starting.fc
      tfc aka display.target.fc 
      200 =: sfc.r, sfc.g, sfc.b 
      200 =: tfc.r, tfc.g, tfc.b
      1 =: display.starting.o.a
      "starting" =: display.state
      f.move.x < 32 => in
    }
    State on_starting_area {
      255 =: display.starting.fc.g
      Clock start_task(500) // 0.5ms
    }
    State started {
      //Clock acquisition_time(100000)
      0 =: display.starting.o.a
      "target" =: display.state
    }
    State miss {
      255 =: display.target.fc.r
      Clock t(50) // 0.05ms
    }
    State success {
      //on_starting_area.acquisition_time.elpased =: target_time_acquisition
      255 =: display.target.fc.g
      Clock t(50) // 0.05ms
    }

    //init -> on_starting_area (display.starting.area.enter)
    init -> on_starting_area (in.true)
    on_starting_area -> init (display.starting.area.leave)    
    on_starting_area -> started (on_starting_area.start_task.tick)
    started -> miss (bg.press)
    started -> success (display.target.area.press)
    miss -> init (miss.t.tick)
    success -> init (success.t.tick)
  }

  //TextPrinter tp
  //control.state => tp.input

}