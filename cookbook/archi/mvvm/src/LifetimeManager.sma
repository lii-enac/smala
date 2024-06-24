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

_define_
LifetimeManager(Process _model)
{
  List controllers
  model aka _model

  Spike about_to_delete
  about_to_delete->(this) {
    for item:this.controllers {
      delete item.control
      delete item.view
      delete item
    }
    delete this.model
    delete this
  }
}