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
*/

use core
use base
use gui

_define_
View2(Process model, int _ty) {
  Translation pos (0, _ty)
  Text t_x (0, 15, "0")
  t_x.wheel.dy -> { t_x.wheel.dy + model.x =: model.x}
  x aka t_x.text
  Text t_y (15, 15, "0")
  t_y.wheel.dy -> { t_y.wheel.dy + model.y =: model.y}
  y aka t_y.text
  Text t_width (30, 15, "0")
  t_width.wheel.dy -> { t_width.wheel.dy + model.width =: model.width}
  width aka t_width.text
  Text t_height (45, 15, "0")
  t_height.wheel.dy -> { t_height.wheel.dy + model.height =: model.height}
  height aka t_height.text
  t_x.x + t_x.width + 5 =:> t_y.x
  t_y.x + t_y.width + 5 =:> t_width.x
  t_width.x + t_width.width + 5 =:> t_height.x
  model.{x,y,width,height}=:>this.{x,y,width,height}
}