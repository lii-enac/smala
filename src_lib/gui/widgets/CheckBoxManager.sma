use core
use base

_define_
CheckBoxManager (Process items, Process init) {
  String value ("")
  init.fsm.initial = "st_selected"
  init.label =: value
  for src_item : items {
  	src_item.selected->{src_item.label =: value}
  	for dst_item : items {
  	  if (&src_item != &dst_item) {
 		src_item.selected->dst_item.unselect
  	  }
  	}
  }
}