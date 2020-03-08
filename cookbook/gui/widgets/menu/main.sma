use core
use exec_env
use base
use display
use gui

import Menu

_main_
Component root
{
  Frame f ("my frame", 0, 0, 500, 500)
  mouseTracking = 1

  NoOutline _()
  FillColor _ (40, 41, 35)
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.width =:> bkg.width
  f.height =:> bkg.height
  
  List items {
    String _ ("Choice 1")
    String _ ("Second choice")
    String _ ("Another choice")
    String _ ("Finally")
  }

  Menu menu (f, items, 100, 10, 100)
  FillColor _ (210, 210, 210)
  FontSize _ (4, 20)
  Text choice (250, 26, "Choice 1")
  menu.selected =:> choice.text
}

run root
run syshook
