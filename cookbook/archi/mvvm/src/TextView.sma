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
*     Stephane Conversy <stephane.conversy@enac.fr>
*
*/
use core
use base
use gui

_define_
TextView(int _ty) {
  // TextView is a textual view of a conceptual rectangle...
  // It displays the rect 4 properties as 4 texts (for x,y,w and h) on the window

  Translation pos (0, _ty)

  // the 4 text graphical objects...
  Text t_x (0, 15, "0")
  Text t_y (15, 15, "0")
  Text t_width (30, 15, "0")
  Text t_height (45, 15, "0")

  // ... laid out horizontally
  t_x.x     + t_x.width     + 5 =:> t_y.x
  t_y.x     + t_y.width     + 5 =:> t_width.x
  t_width.x + t_width.width + 5 =:> t_height.x
}