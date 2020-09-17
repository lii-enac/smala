use core
use base
use display
use gui

import MyComponent
import Menu

_main_
Component root
{
  Frame f ("my frame", 0, 0, 700, 400)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1

  NoOutline _()
  FillColor _ (200, 200, 200)
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.width =:> bkg.width
  f.height =:> bkg.height

  List items {
    String _ ("width")
    String _ ("height")
    String _ ("color/r")
  }

  Menu menu (f, items, 370, 10, 100)

  FillColor _ (100, 100, 100)
  Rectangle button_asc (480, 10, 100, 20, 5, 5)
  FillColor _ (200, 200, 200)
  TextAnchor _ (1)
  Text order (560, 25, "descending")
  order.width + 10 =: button_asc.width
  button_asc.width / 2 + button_asc.x =: order.x

  List mylist {
    MyComponent r1 (10, 10, 100, 30, 0)
    MyComponent r2 (10, 80, 30, 50, 255)
    MyComponent r3 (10, 150, 120, 20, 160)
    MyComponent r4 (10, 230, 200, 60, 80)
  }

  Sorter sorter (mylist, "width")
  menu.selected => sorter.spec

  FSM fsm {
    State descending {
      0 =: sorter.ascending
    }
    State ascending {
      1 =: sorter.ascending
    }
    ascending->descending (button_asc.press)
    descending->ascending (button_asc.press)
  }
  fsm.state =:> order.text
  button_asc.press -> sorter.sort
  sorter.sort->(root) {
    int y = 10
    for (int i = 1; i <= root.mylist.size; i++) {
      root.mylist.[i].move_y_to = y
      y = y + 70
    }
  }
}

