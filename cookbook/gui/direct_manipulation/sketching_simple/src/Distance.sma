/*
*  Smala cookbook sketching_simple
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2018-2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Nicolas Saporito <nicolas.saporito@enac.fr>
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*
*/
use core
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