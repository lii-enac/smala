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
import view_model.RectViewModel

_define_
ViewModelManager ()
{
  List view_models_list
  //ProcessCollector view_models_list

  Ref view_model_to_delete (null)
  Ref model_to_delete (null)

  Ref selected_VM (null)

  ModelManager model_manager ()

  Spike new_rectangle
  Spike delete_rectangle

  new_rectangle -> model_manager.add_new_rectangle

  delete_rectangle -> model_manager.delete_last_rectangle

  // When a model is added to the list
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

  // When a model is removed from the list
  model_manager.models_list.$removed -> na_models_list_removed:(this) {
    model = getRef (&this.model_manager.models_list.$removed)
    //model = getRef (&src)
    if (&model != null) {
      print ("VM models_list_removed (avant): " + this.view_models_list.size + " VMs")
      for view_model : this.view_models_list {
        if (&view_model.model == &model) {
          // We cannot delete the view model yet
          //delete view_model
          // Store references on the model and the VM to delete
          setRef (this.view_model_to_delete, view_model)
          setRef (this.model_to_delete, model)
          
          // Only remove from list
          remove view_model from this.view_models_list
          break
        }
      }
      print ("VM models_list_removed (apres): " + this.view_models_list.size + " VMs")
    }
  }

  // All views have been deleted, now we can delete the view model
  na_models_list_removed -> na_views_have_been_deleted:(this) {
    print ("all Views have been deleted...we can delete the VM")
    view_model_to_delete = getRef (this.view_model_to_delete)
    if (&view_model_to_delete != null) {
      setRef (this.view_model_to_delete, null)
      // We cannot delete the model yet
      //delete view_model_to_delete.model
      
      // Delete the view model (and free memory)
      delete view_model_to_delete
    }
  }

  // View Model has been deleted, now we can delete the model
  // Could be done in ModelManager
  na_views_have_been_deleted -> na_vm_has_been_deleted:(this) {
    print ("VM has been deleted...we can delete the model")
    model_to_delete = getRef (this.model_to_delete)
    if (&model_to_delete != null) {
      setRef (this.model_to_delete, null)

      // Delete the model (and free memory)
      delete model_to_delete
    }
  }
}