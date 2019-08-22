use core
use base
use gui

_main_
Component root {
  Frame f ("my frame", 0, 0, 400, 600)

  FillColor c (0, 0, 0)
  
  SwitchRange swr (0) {
    _ [0, 33[ {
      210 =: c.r
      34  =: c.g
      45  =: c.b
    }
    _ [33, 66[
    {
      255 =: c.r
      191 =: c.g
      0   =: c.b
    }
    _ [66, 100]
    {
      25  =: c.r
      136 =: c.g
      35  =: c.b
    }
  }

  Rectangle r (0, 0, 70, 50, 5, 5)
  r.y => swr.state
  FSM fsm {
    State idle
    State drag {
      Double init_y (0)
      BoundedValue bv (0, 100, 0)
      r.press.y =: init_y
      f.move.y - init_y => bv.input
      bv.result => r.y
    }
    idle->drag (r.press)
    drag->idle (f.release)
  }
}


run root
run syshook