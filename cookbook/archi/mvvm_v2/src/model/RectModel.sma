/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023-2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use gui

_define_
RectModel (double _x, double _y, double _w, double _h)
{
  TextPrinter tp

  Double x (_x)
  Double y (_y)
  Double width (_w)
  Double height (_h)

  Double surface_area (0)
  width * height =:> surface_area

  //"Model: surface area = " + surface_area =:> tp.input
}