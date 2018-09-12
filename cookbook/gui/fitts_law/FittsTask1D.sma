use core
use base
use gui

_define_
FittsTask1D(Component f) {
  // make cursor explicit? => modelling helps to fill holes/gaps

  Int target_width(10)
  Int target_distance(500)
  Int target_time_acquisition(0)

  FillColor bgfc(0,0,0)
  Rectangle bg(0,0,1,1,0,0)
  f.width => bg.width
  f.height => bg.height

  Switch display(starting_area) {
    Component starting_area {
      Int width(32)
      Translation t(0,0)
      //width/2 => t.tx
      //f.height/2 => t.ty
      FillOpacity o(1)
      FillColor fc(200, 200, 200) // lightgray
      Rectangle area (0,0, 0,0, 0, 0)
      //f.width-width/2 => area.x
      (f.height-width)/2 => area.y
      width => area.width
      width => area.height
    }
    Component target {
      FillColor fc(200, 200, 200) // lightgray
      Rectangle area (0, 0, 0,0 , 0, 0)
      target_distance-target_width/2 => area.x
      target_width => area.width
      f.height => area.height
    }
  }

  FSM status {
    State init {
      200 =: display.starting_area.fc.r, display.starting_area.fc.g, display.starting_area.fc.b 
      200 =: display.target.fc.r, display.target.fc.g
      1 =: display.starting_area.o.a
      "starting_area" =: display.state
    }
    State on_starting_area {
      255 =: display.starting_area.fc.g
      Clock start_task(500) // 0.5ms
    }
    State started {
      Clock acquisition_time(100000)
      0 =: display.starting_area.o.a
      "target" =: display.state
    }
    State miss {
      255 =: display.target.fc.r
      Clock t(50) // 0.05ms
    }
    State success {
      255 =: display.target.fc.g
      Clock t(50) // 0.05ms
    }

    init -> on_starting_area (display.starting_area.area.enter)
    on_starting_area -> init (display.starting_area.area.leave)    
    on_starting_area -> started (on_starting_area.start_task.tick)
    started -> miss (bg.press)
    started -> success (display.target.area.press)
    miss -> init (miss.t.tick)
    success -> init (success.t.tick)
  }
  //TextPrinter tp
  //status.state => tp.input
}