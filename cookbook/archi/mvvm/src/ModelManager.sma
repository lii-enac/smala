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

import RectModel

_define_
ModelManager ()
{
  //Incr incr (0)
  List models_list
  //ProcessCollector models_list

  Spike add_new_rectangle
  Spike delete_last_rectangle

  add_new_rectangle -> na_add_new_rectangle:(this) {
    print ("M add_new_rectangle (avant) " + this.models_list.size + " models")
    Process model = RectModel (this.models_list, "", 50, 50, 100, 70)
    print ("M add_new_rectangle (apres) " + this.models_list.size + " models")
  }

  delete_last_rectangle -> na_delete_last_rectangle:(this) {
    print ("M delete_last_rectangle (avant) " + this.models_list.size + " models")
    if (this.models_list.size > 0) {
      int size = getInt (this.models_list.size)
      delete this.models_list.[size]
    }
    print ("M delete_last_rectangle (apres) " + this.models_list.size + " models")
  }
}