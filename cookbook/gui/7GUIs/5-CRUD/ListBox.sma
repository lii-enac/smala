/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017-2026)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */
use core
use base
use display
use gui

// import gui.widgets.AbstractBox
import gui.widgets.VBox


_action_
action_select_item (Process src, Process self)
{
  item = find (&src, "..")
  // print ("Select item " + item.text)

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


_action_
action_unselect_item (Process src, Process self)
{
  selected_item = getRef (self.selected_item)
  if (&selected_item != null) {
    notify selected_item.unselect
  }

  setRef (self.selected_item, null)
}


_define_
// ListBox (Process _model) inherits AbstractBox () {
// ListBox (Process _models) inherits VBox () {
ListBox () inherits VBox () {
  // models aka _models

  this.space = 2

  Ref selected_item (null)
  Spike reset_selection

  Component bindings

  Component bg {
    // FillOpacity op (0.5)
    FillColor fill_c (#FFFFFF)

    OutlineWidth _ (1)
    OutlineColor outln_c (#AAAAAA)

    Rectangle r (0, 0, 0, 0, 5, 5)
    
    (this.preferred_width == -1) || (this.preferred_width < this.min_width) ? this.min_width : this.preferred_width =:> r.width
    this.preferred_height == -1 ? this.min_height : this.preferred_height =:> r.height
  }

  // TextPrinter tp
  // "Max of items min W = " + this.ml.output + " -- min W " + this.min_width + " -- preferred W " + this.preferred_width + " -- container W " + this.container_width =:> tp.input


  // ProcessCollector "items" in class AbstractBox:
  // - RefProperty _add is not set
  // - sub process "add" is not activated when an item is added

  // We can NOT bind to this.items.add !
  // this.items.add -> na_item_added:(this) {
  //   item = getRef (&this.items.add)
  
  // Can we bind to this.items.rm ?
  // this.items.rm -> na_item_removed:(this) {
  //   item = getRef (&this.items.rm)


  NativeAction na_select_item (action_select_item, this, 1)
  NativeAction na_unselect_item (action_unselect_item, this, 1)

  this.items.size -> na_items_size:(this) {
    // print ("Size changed: " + this.items.size + " items in the list box")

    for item : this.items {
      if (!item._is_binded) {
        // print ("Add binding on " + item.text)
        item._is_binded = true

        addChildrenTo this.bindings {
          item.select -> this.na_select_item
          // item.unselect -> this.na_unselect_item
        }
      }
    }
  }

  // (selected_item.is_null == 0) && reset_selection -> na_reset_selection:(this) {
  //   setRef (this.selected_item, null)
  // }
  // (selected_item.is_null == 0) && reset_selection -> {
  //   null =: selected_item
  // }
  reset_selection -> na_unselect_item


//   ui.width =: this.min_width
//   this.preferred_width == -1 ? ui.width : this.preferred_width =: this.preferred_width
//   ui.height =: this.min_height
//   this.preferred_height == -1 ? ui.height : this.preferred_height =: this.preferred_height

  // will be moved at the end, serves as a marker for additional components
  moveChild this.remaining >>
}