/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2021-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base

import core.ontology.process
import core.tree.container
import base.process_handler

_action_
action_selected_item (Process src, Process self)
{
  item = find (&src, "..")
//   print ("Select item " + item.label)

  previous_selected_item = getRef (self.selected_item)
  if (&previous_selected_item != null) {
    // print ("Previous selected item " + previous_selected_item.text + "\n")
    notify previous_selected_item.unselect
  }
  // else {
  //   print("NO previous selected item \n")
  // }

  setRef (self.selected_item, item)
}

_define_
CheckBoxManager (Process _collection, int _init_index) {
	collection aka _collection
	Int init_index (_init_index)

  Ref selected_item (null)
  DerefString label_of_selected_value (selected_item, "label", DJNN_GET_ON_CHANGE)
  value aka label_of_selected_value.value

  Component bindings

  NativeAction na_selected_item (action_selected_item, this, 1)

  collection.size -> na_items_size:(this) {
    print ("Size changed: " + this.collection.size + " items in the list box")

	int i = 1
    for item : this.collection {
      if (!item._is_binded) {
        // print ("Add binding on " + item.label)
        item._is_binded = true

        addChildrenTo this.bindings {
          // FIXME: memory leak when we delete an item
          item.selected -> this.na_selected_item
          // item.unselected -> this.na_unselect_item
        }
      }

	  	if (i == this.init_index) {
			notify item.press // = UGLY but = selected for this checkbox
		}
		i++
    }
  }
}