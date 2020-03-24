use core
use base
use display
use gui

_main_
Component root
{
  Frame f ("my frame", 0, 0, 500, 300)
  Exit ex (0, 1)
  f.close -> ex
  FillColor _ (100, 100, 100)
  Rectangle r1 (10, 30, 200, 100, 0, 0)
  Rectangle r2 (250, 30, 150, 70, 0, 0)

  FillColor _ (200, 150, 150)
  Rectangle r3 (10, 140, 70, 30, 3, 3)
  
  RefProperty current (r1)
  
  AssignmentSequence set_r1 (1) {
    r1 =: current
  }
  AssignmentSequence set_r2 (1) {
    r2 =: current
  }

  Deref deref (current, "press")
  AssignmentSequence set_move (1) {
    "move" =: deref.path
  }
  r3.press->set_move
  Incr i (1)
  deref.activation -> i

  r1.press->set_r1
  r2.press->set_r2

  FillColor _ (Black)
  Text _ (10, 12, "Click alternately two times on the left and right grey rectangles,")
  Text _ (10, 26, "then click on the red one and click and move on the grey ones")
  Text t (10, 200, "")
  i.state =:> t.text
}

