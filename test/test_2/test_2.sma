use core
use base
use display
use gui

_main_
Component root
{
  print ("\n/****** TEST 2 *******/\n")
  print ("\nCheck that on each click on the black rectangle, one and only one FSM transition is executed.\n")
  print ("The console should print alternately small/large\n")

  Frame f ("my frame", 0, 0, 500, 500)
  Rectangle r (100, 100, 100, 60, 5, 5)
  TextPrinter tp
  FSM fsm {
    State large
    State small
    large->small (r.press)
    small->large (r.press)
  }
  fsm.state=>tp.input
}

run root
run syshook
