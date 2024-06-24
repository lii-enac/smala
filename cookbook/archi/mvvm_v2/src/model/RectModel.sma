/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/

use core
use gui

_define_
RectModel (int _x, int _y, int _w, int _h)
{
  TextPrinter tp

  Int x (_x)
  Int y (_y)
  Int width (_w)
  Int height (_h)

  Double surface_area (0)
  width * height =:> surface_area

  //"Model: surface area = " + surface_area =:> tp.input
}