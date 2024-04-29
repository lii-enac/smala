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
RectViewModel (Process _model) {
  model aka _model

  TextPrinter tp

  Int x ($model.x)
  Int y ($model.y)
  Int width ($model.width)
  Int height ($model.height)

  // update our view model if needed
  model.x =?> x
  model.y =?> y
  model.width =?> width
  model.height =?> height

  // update model if needed (from interactions on the view)
  x =?> model.x
  y =?> model.y
  width =?> model.width
  height =?> model.height

  Bool is_presssed (false)
  Bool is_selected (false)

  Double surface_area (0)
  //_model.width * _model.height =:> surface_area
  width * height =?> surface_area

  //"VM: surface area = " + surface_area =:> tp.input

  Spike close
}