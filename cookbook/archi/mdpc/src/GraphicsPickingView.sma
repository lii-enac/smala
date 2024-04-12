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
*     Stephane Conversy <stephane.conversy@enac.fr>
*/

use core
use base
use gui

_define_
GraphicsPickingView() {
  NoFill _
  NoOutline _
  PickFill _

  Int border (5)

  Rectangle r (0,0,0,0)
  Rectangle left (0,0,0,0)
  Rectangle right (0,0,0,0)
  Rectangle top (0,0,0,0)
  Rectangle bottom (0,0,0,0)

  border =:> left.width, right.width, top.height, bottom.height

  r.x =:> top.x, bottom.x
  r.y =:> top.y
  r.width =:> top.width, bottom.width
  r.height =:> left.height, right.height

  r.x =:> left.x
  r.x + r.width - border*2 =:> right.x
  r.y =:> left.y, right.y
  r.y + r.height - border*2 =:> bottom.y

}