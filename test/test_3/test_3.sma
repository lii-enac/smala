use core
use base
use display
use gui

_main_
Component root
{
  print ("\n/****** TEST 3 *******/\n")
  print ("\nCheck that on frame resizing, the FSM moves from large to small state.\n")
  print ("The console should print alternately small/large when crossing le line\n")

  Frame f ("my frame", 0, 0, 500, 500)
  OutlineColor _ (0, 0, 0)
  Line _ (250, 0, 250, 500)
  TextPrinter tp
  Bool test (1)
  f.width > 250 => test
  FSM fsm {
    State large
    State small
    large->small (test.false)
    small->large (test.true)
  }
  fsm.state => tp.input
}

run root
run syshook
