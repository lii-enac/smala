use core
use base
use display
use gui

_main_
Component root
{
  print ("\n/****** TEST 1 *******/\n")
  print ("\nCheck that on each click on the black rectangle, and only in this case, the number of clicks is printed in the console\n")

  Frame f ("my frame", 0, 0, 500, 500)
  Rectangle r (100, 100, 100, 60, 5, 5)
  
  Incr inc (1)
  TextPrinter tp
  inc.state => tp.input
  r.press -> inc
}

run root
run syshook
