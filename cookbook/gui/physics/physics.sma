use core
use base
use gui
use physics

_native_code_
%{
#include <cmath>
%}

_main_
Component root
{
  Frame f ("my frame", 0, 0, 1000, 1000)
  Exit ex (0, 1)
  f.close -> ex

  World w (0, 0, -9.81)
  Sphere s (1.5, 0, 10, 0.2, 1)
  Sphere s1 (1.52, 0, 9.5, 0.2, 1)

  Plane plane (0, 0, 1, 0)


  FSM fsm {
    State idle
    State running {
      Clock cl (10)
      cl.tick->w.step
    }
    idle->running (f.press)
    running->idle (f.press)
  }

  FillColor fc1 (0, 0, 0)
  Circle cir (0, 0, 20)
  Text tp (600, 100, "")
  min (abs($s.dz) * 20, 255.0) => fc1.b
  f.height - (s.z*100) =:> cir.cy
  s.x * 100 =:> cir.cx

  FillColor fc2 (0, 0, 0)
  min (abs($s1.dz) * 20, 255.0) => fc2.b
  Circle cir2 (0, 20, 20)
  f.height - (s1.z*100) =:> cir2.cy
  s1.x * 100 =:> cir2.cx
}
