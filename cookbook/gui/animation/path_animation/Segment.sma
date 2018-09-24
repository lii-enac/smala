use core
use base

_define_
Segment (Process p1, Process p2)
{
  Double input (0)
  Double range_x (0)
  Double range_y (0)

  Double x (0)
  Double y (0)
  
  p2.x - p1.x => range_x
  p2.y - p1.y => range_y

  p1.x + input * range_x => x
  p1.y + input * range_y => y
}