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

  Int x ($model.x)
  Int y ($model.y)
  Int width ($model.width)
  Int height ($model.height)

  Bool is_presssed (false)
  Bool is_selected (false)

  Double surface_area (0)
  _model.width * _model.height =:> surface_area

  Spike close
}