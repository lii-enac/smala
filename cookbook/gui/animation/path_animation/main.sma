use core
use exec_env
use base
use display
use gui

import Segment


_main_
Component root {
  Frame f ("f", 0, 0, 800, 800)
  NoOutline _
  FillColor _ (50, 50, 50)
  Rectangle bg (0, 0, 0, 0, 0, 0)
  f.width =:> bg.width
  f.height =:> bg.height

  OutlineColor _ (200, 150, 150)
  Polyline pl {
    Point p1 (100, 50)
    Point p2 (400, 200)
    Point p3 (50, 250)
  }

  FillColor _ (200, 150, 150)
  Clock cl (20)
  Incr inc (1)
  inc.delta = 0.02
  cl.tick -> inc

  Circle c (0, 0, 10)
  SwitchList sl {
    Component i1 {
      0 =: inc.state
      Segment s1 (pl.p1, pl.p2)
      s1.x =:> c.cx
      s1.y =:> c.cy
      inc.state =:> s1.input
    }
    Component i2 {
      0 =: inc.state
      Segment s2 (pl.p2, pl.p3)
      s2.x =:> c.cx
      s2.y =:> c.cy
      inc.state =:> s2.input
    }
    Component end
  }
  Bool end (0)
  inc.state > 1 =:> end
  end.true -> sl.next

}

run root
run syshook