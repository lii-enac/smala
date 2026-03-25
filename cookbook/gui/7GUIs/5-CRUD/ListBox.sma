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
  self.is_selected_item = true
}


// _action_
// action_model_unselected (Process src, Process self)
// {
//   // This action is called when user unselect or clicks on background
//   // --> src is not always track_model
//   //track_model = find (&src, "..")
  
//   if (self.is_selected_item)
//   {
//     selected_model = getRef (self.selected_model)
//     if (&selected_model != null)
//     {
//       //print ("Previous selected flight " + selected_model.callsign + "\n")
//       selected_model.is_selected = false
//     }

//     setRef (self.selected_model, null)
//     self.is_selected_item = false
//   }
// }



_define_
// ListBox (Process _model) inherits AbstractBox () {
// ListBox (Process _models) inherits VBox () {
ListBox () inherits VBox () {
  // models aka _models

  this.space = 2

  Bool is_selected_item (false)
  Ref selected_item (null)

  Component bindings

  Component bg {
    // FillOpacity op (0.5)
    FillColor fill_c (#FFFFFF)

    OutlineWidth _ (2)
    OutlineColor outln_c (#FF0000)

    Rectangle r (0, 0, 0, 0, 5, 5)
    
    (this.preferred_width == -1) || (this.preferred_width < this.min_width) ? this.min_width : this.preferred_width =:> r.width
    this.preferred_height == -1 ? this.min_height : this.preferred_height =:> r.height
  }

  TextPrinter tp
  // "Max of min width = " + this.ml.output =:> tp.input
  "Max of items min W = " + this.ml.output + " -- min W " + this.min_width + " -- preferred W " + this.preferred_width + " -- container W " + this.container_width =:> tp.input


  // ProcessCollector "items" in class AbstractBox: sub process "add" is not activated when an item is added
  // We can NOT bind to this.items.add !
  // this.items.add -> na_item_added:(this) {
  //   item = getRef (&this.items.add)
  
  // Can we bind to this.items.rm ?
  // this.items.rm -> na_item_removed:(this) {
  //   item = getRef (&this.items.rm)


  NativeAction na_select_item (action_select_item, this, 1)

  this.items.size -> na_items_size:(this) {
    print ("Items size changed " + this.items.size)

    for item : this.items {
      if (!item._is_binded) {
        print ("Add binding on " + item.text)
        item._is_binded = true

        addChildrenTo this.bindings {
          item.select -> this.na_select_item
        }
      }
    }
  }

//   Translation offset (0, 0)
//   Text ui (0, 0, _label)
//   text aka ui.text
//   this.height/2 + ui.ascent/2 - 1 =:> offset.ty

//   ui.width =: this.min_width
//   this.preferred_width == -1 ? ui.width : this.preferred_width =: this.preferred_width
//   ui.height =: this.min_height
//   this.preferred_height == -1 ? ui.height : this.preferred_height =: this.preferred_height
}