use core
use base
use display
use gui

_define_
dnd(Process frame, Process t, Process mobile, Process trash)
{

  Spike deleteFile -> {
    TextPrinter tp
    "yep" =: tp.input
  }

  FSM fsm {
    State idle
    State dragging {
      Int x(0)
      Int y(0)
      frame.move.x - t.tx =: x
      frame.move.y - t.ty =: y 
      frame.move.x - x =:> t.tx
      frame.move.y - y =:> t.ty
    }
    State inTrash
    idle -> dragging (mobile.press)
    dragging -> idle (frame.release)
    dragging -> inTrash (trash.icon.enter)
    inTrash -> dragging (trash.icon.leave)
    inTrash -> idle (frame.release)
  }
}
