use core
use base
use display
use gui

import gui.widgets.IWidget

_native_code_
%{
int
isFrame (Process* p) {
  return p->get_process_type () == WINDOW_T;
}
%}

_define_
WidgetContainer (Process parent) inherits IWidget (parent)
{
	Translation pos (0, 0)
  x aka pos.tx
  y aka pos.ty
  if (isFrame (parent)) {
    parent.width =:> this.req_width
    parent.height =:> this.req_height
    this.min_width =:> parent.min_width
    this.min_height =:> parent.min_height
  }
}
