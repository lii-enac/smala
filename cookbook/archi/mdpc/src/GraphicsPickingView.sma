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

  //FillColor _(255,0,0)
  Rectangle r (0,0,0,0) // center
  // FillOpacity _(0.5)
  // FillColor _(0,255,0)
  Rectangle left (0,0,0,0)
  Rectangle right (0,0,0,0)
  Rectangle top (0,0,0,0)
  Rectangle bottom (0,0,0,0)

  x aka r.x
  y aka r.y
  width aka r.width
  height aka r.height

  Double border (5)
  Double invtborder (0)

  border * 2 =:> invtborder // make interactive border zoom-independant
  // ScreenToLocal m (r)
  // border =:> m.inX
  // m.outX =:> invtborder
  // border =:> m.inY
  // sqrt(m.outX*m.outX+m.outY*m.outY) =:> invtborder
  // m.outY =:> mdy
  
  // TextPrinter tp
  // invtborder =:> tp.input


                   invtborder =:> top.height, bottom.height, left.width,  right.width

                          r.x =:> top.x,      bottom.x,      left.x
                          r.y =:> top.y,                     left.y,      right.y
                      r.width =:> top.width,  bottom.width
                     r.height =:>                            left.height, right.height

  r.x + r.width  - invtborder =:>                                         right.x
  r.y + r.height - invtborder =:>             bottom.y

}