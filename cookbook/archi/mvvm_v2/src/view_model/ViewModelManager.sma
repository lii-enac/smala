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

_native_code_
%{
#include <assert.h>
%}

// import ModelManager
import RectViewModel


_action_
action_close_rectangle (Process src, Process self)
{
  view_model = find (&src, "..")
  if (&view_model != null) {
    // just remove from list, deletion will be triggered
    remove view_model.model from self.model_manager.models_list
  }
}


_define_
ViewModelManager (Process model_manager_)
{
  List view_models_list
  model_manager aka model_manager_

  NativeAction na_close_rectangle (action_close_rectangle, this, 1)

  // When a model is added to the list
  model_manager.models_list.$added -> na_models_list_added:(this) {
    //print ("VM models_list_added (before): " + this.view_models_list.size + " VMs")
    model = getRef (&this.model_manager.models_list.$added)
    assert (&model != null)
    Process view_model = RectViewModel (this.view_models_list, "", model)
    // add a callback to the cross button to allow rectangle deletion
    addChildrenTo this {
      //view_model.is_selected -> ...
      this.view_models_list.[this.view_models_list.size].close -> this.na_close_rectangle
    }
    //print ("VM models_list_added (after): " + this.view_models_list.size + " VMs")
  }

  // When a model is removed from the list
  Ref view_model_to_delete (null)
  Ref model_to_delete (null)

  model_manager.models_list.$removed -> na_models_list_removed:(this) {
    //print ("VM models_list_removed (before): " + this.view_models_list.size + " VMs")
    model = getRef (&this.model_manager.models_list.$removed)
    assert (&model != null)

    // find the view_model corresponding to this model
    Process view_model = null
    for vm : this.view_models_list {
      if (&vm.model == &model) {
        view_model = &vm
        break
      }
    }
    assert (&view_model != null)
    
    // We cannot delete the view_model yet because views still point to them
    //delete view_model
    // ...so store references on the model and the view_model to delete later
    setRef (this.view_model_to_delete, view_model)
    setRef (this.model_to_delete, model)
    // ...and just remove it from the list:
    // this will trigger deletion of views on this view_model
    remove view_model from this.view_models_list
    //print ("VM models_list_removed (after): " + this.view_models_list.size + " VMs")
  }

  // All views have been deleted, now we can safely delete the view model
  na_models_list_removed -> na_views_have_been_deleted:(this) {
    // print ("all Views have been deleted...we can delete the VM")
    view_model_to_delete = getRef (this.view_model_to_delete)
    assert (&view_model_to_delete != null)
    setRef (this.view_model_to_delete, null)
    // We cannot delete the model just yet because view_model still has bindings to it
    //delete view_model_to_delete.model
    delete view_model_to_delete
  }

  // view_model has been deleted, now we can safely delete the model
  // (could be done in the ModelManager)
  na_views_have_been_deleted -> na_vm_has_been_deleted:(this) {
    //print ("VM has been deleted...we can delete the model")
    model_to_delete = getRef (this.model_to_delete)
    assert (&model_to_delete != null)
    setRef (this.model_to_delete, null)
    delete model_to_delete
  }
}