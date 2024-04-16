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

import ModelManager
import RectViewModel

_define_
ViewModelManager ()
{
  List view_models_list

  Ref selected_VM (null)

  ModelManager model_manager ()

  Spike new_rectangle
  Spike delete_rectangle

  new_rectangle -> model_manager.add_new_rectangle

  delete_rectangle -> model_manager.delete_last_rectangle

  model_manager.models_list.$added -> na_models_list_added:(this) {
    model = getRef (&this.model_manager.models_list.$added)
    //model = getRef (&src)
    if (&model != null) {
      print ("VM models_list_added (avant): " + this.view_models_list.size + " VMs")
      Process view_model = RectViewModel (this.view_models_list, "", model)
      //view_model.is_selected ->
      print ("VM models_list_added (apres): " + this.view_models_list.size + " VMs")
    }
  }

  model_manager.models_list.$removed -> na_models_list_removed:(this) {
    model = getRef (&this.model_manager.models_list.$removed)
    //model = getRef (&src)
    if (&model != null) {
      print ("VM models_list_removed (avant): " + this.view_models_list.size + " VMs")
      for view_model : this.view_models_list {
        if (&view_model.model == &model) {
          delete view_model
          break
        }
      }
      print ("VM models_list_removed (apres): " + this.view_models_list.size + " VMs")
    }
  }
}