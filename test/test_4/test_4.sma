use core
use base
use display
use gui

_main_
Component root
{
  print ("\n/****** TEST 4 *******/\n")
  print ("\nCheck that on frame resizing, the FSM moves from large to small state.\n")
  print ("The console should print alternately small/large followed by the current width of the frame\n")
  print ("When in small state, the rectangle should move to the left side of the frame.\n")
  print ("When in large state, the rectangle should stay in the middle of the frame.\n")
  print ("Verify that the printed message is on time and coherent (no large + 0 for example) \n")

  Frame f ("my frame", 0, 0, 500, 500)
  Rectangle r (100, 100, 100, 60, 5, 5)
  Double width (0)
  Switch sw (large) {
    Component large {
      f.width / 2 - r.width/2 =:> r.x
      f.width =:> width
    }
    Component small {
      0 =: r.x
      0 =: width
    }
  }
  TextPrinter tp
  Bool test (1)
  f.width > 250 => test
  FSM fsm {
    State large
    State small
    large->small (test.false)
    small->large (test.true)
  }
  fsm.state => sw.state
  isString(fsm.state + " " + width) => tp.input
}

run root
run syshook
