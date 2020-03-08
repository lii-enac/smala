use core
use exec_env
use base

_define_
/* p1 and p2 must have x and y children */
Distance (Process p1, Process p2)
{
  Sqrt sqrt (0)
  result aka sqrt.output

  Pow pow_x (0, 2)
  Pow pow_y (0, 2)

  p2.x - p1.x =:> pow_x.base
  p2.y - p1.y =:> pow_y.base
  
  pow_x.result + pow_y.result =:> sqrt.input
}