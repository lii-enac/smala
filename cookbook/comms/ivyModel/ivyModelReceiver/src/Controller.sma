/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use gui

_define_
Controller(Process _model, Process _view)
{
  model aka _model
  view aka _view
  
  Spike about_to_delete

  Component control {
    model.{info1, info2, info3, info4, info5, info6, info7} =:> view.{info1, info2, info3, info4, info5, info6, info7}
  }

  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this.model
    delete this
  }
}