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
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/

use core
use base
use gui

_define_
RectView (Process _view_model) {
  vm aka _view_model

  OutlineColor outline (Black)
  OutlineWidth w (5)
  
  FillColor fill (Red)

  Rectangle r (0, 0, 0, 0)

  vm.x =:> r.x
  vm.y =:> r.y
  vm.width =:> r.width
  vm.height =:> r.height

}