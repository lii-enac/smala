/*
*  Smala cookbook path_animation
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2018-2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     jeremie-garcia <jeremie.garcia@enac.fr>
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*
*/
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
  
  p2.x - p1.x =:> range_x
  p2.y - p1.y =:> range_y

  p1.x + input * range_x =:> x
  p1.y + input * range_y =:> y
}