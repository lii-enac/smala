use core
use base
use display
use gui

_main_
Component root
{
  Frame f ("my frame", 0, 0, 500, 300)

  FillColor _ (100, 100, 100)
  Rectangle r1 (10, 10, 200, 100, 0, 0)
  Rectangle r2 (250, 10, 150, 70, 0, 0)

  FillColor _ (0, 0, 0)
  Text width (200, 150, "")
  Text height (200, 165, "")
  RefProperty current (r1)
  
  set_r1 = r1 =: current : 1
  set_r2 = r2 =: current : 1

  current.$value.width => width.text
  current.$value.height => height.text

  r1.press->set_r1
  r2.press->set_r2
}

run root
run syshook
