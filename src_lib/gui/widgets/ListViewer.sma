/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
use core
use base
use display
use gui

_native_code_
%{
static int has_view_item (Process *model_item, Process *view_list) {
  //string id_model_item = ((djnn::TextProperty*)xmodel_item->find_child("id"))->get_value ();
  for (auto p : ((djnn::List*)view_list)->children ()) {
    if (model_item == ((djnn::RefProperty*)p->find_child("model"))->get_value ()) {
      return 1;
    }
  }
  return 0;
}

static int has_model_item (Process *view_item, Process *model_list) {
  Process* item = ((djnn::RefProperty*)view_item->find_child("model"))->get_value ();
  for (auto p : ((djnn::List*)model_list)->children ()) {
    if (p == item) {
      return 1;
    }
  }
  return 0;
}

static void setIsModel (Process *p, int v) {
    if (p)
      p->set_is_model (v);
}

%}

_define_
ListViewer (Process _model, Process _prototype) {
  Int space (5)
  Int orientation (0) // 0 - vertical, 1- horizontal
  Translation pos (0, 0)
  x aka pos.tx
  y aka pos.ty

  RefProperty model (_model)
  Deref added (model, "$added")
  Deref removed (model, "$removed")

  prototype aka _prototype
  item_height aka _prototype.height
  item_width aka _prototype.width

  List view_items

  added.activation -> add_items: (this) {
    if (this.model == 0) {
      return
    }
    int x = 0
    int y = 0
    int size = getInt (this.view_items.size)
    if (this.orientation == 0) {
      y = size * (this.item_height + this.space)
    } else {
      x = size * (this.item_width + this.space)
    }
    model = getRef (this.model)
    for item : model {
      if (!has_view_item (item, this.view_items)) {
        new_item = clone (this.prototype)
        if (this.orientation == 0) {
          new_item.y = y
          y += this.item_height + this.space
        } else {
          new_item.x = x
          x += this.item_width + this.space
        }
        setRef (new_item.model, item)
        addChildrenTo this.view_items {
          new_item
        }
      }
    }
  }

  setIsModel (add_items, 0)

  model != 0 -> add_items
  removed.activation -> remove_items: (this) {
    list_model = getRef (this.model)
    for item : this.view_items {
      if (has_model_item (item, list_model) == 0) {
          setRef (item.model, NULL)
          graph_exec () // bad trick - this is to be sure that every deref to the model in the view has been cleared before the model is deleted
          delete item
      }
    }
    int y = 0
    int x = 0
    for item : this.view_items {
      if (this.orientation == 0) {
        item.y = y
        y += this.item_height + this.space
      } else {
        item.x = x
        x += this.item_width + this.space
      }
    }
  }
}
