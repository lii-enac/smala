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

_define_
TextsListView (Process _view_model_manager)
{
  view_model_manager aka _view_model_manager

  OutlineColor out_c (#000000)
  OutlineWidth out_w (1)
  NoFill _

  Rectangle bg (0, 0, 100, 100)
  x aka bg.x
  y aka bg.y
  width aka bg.width
  height aka bg.height

}