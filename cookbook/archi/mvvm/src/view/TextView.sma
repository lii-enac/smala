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
TextView (Process _view_model, int _delta_y) {
  vm aka _view_model

  //TextPrinter tp

  Translation pos (10, _delta_y)

  Text t_x (0, 15, "0")
  //x aka t_x.text

  Text t_y (15, 15, "0")
  //y aka t_y.text

  Text t_width (30, 15, "0")
  //width aka t_width.text

  Text t_height (45, 15, "0")
  //height aka t_height.text

  // Layout
  t_x.x + t_x.width + 5 =:> t_y.x
  t_y.x + t_y.width + 5 =:> t_width.x
  t_width.x + t_width.width + 5 =:> t_height.x

  // update the view whenever the view model changes
  vm.x =:> t_x.text
  vm.y =:> t_y.text
  vm.width =:> t_width.text
  vm.height =:> t_height.text

  // update view model from interactions on the view
  vm.x + t_x.wheel.dy => vm.x
  vm.y + t_y.wheel.dy => vm.y
  vm.width + t_width.wheel.dy => vm.width
  vm.height + t_height.wheel.dy => vm.height
  
}