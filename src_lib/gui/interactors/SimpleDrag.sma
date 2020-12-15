use core
use base

_define_
SimpleDrag (Process ref_to_obj, Process frame)
{
  Deref press (&ref_to_obj, "press")
  DerefDouble press_x (&ref_to_obj, "press/x", DJNN_GET_ON_CHANGE)
  DerefDouble press_y (&ref_to_obj, "press/y", DJNN_GET_ON_CHANGE)

  DerefDouble x (&ref_to_obj, "x", DJNN_GET_ON_CHANGE)
  DerefDouble y (&ref_to_obj, "y", DJNN_GET_ON_CHANGE)

  FSM fsm {
    State idle
    State dragging {
      Int off_x (0)
      Int off_y (0)
      press_x.value - x.value =: off_x
      press_y.value - y.value =: off_y
      frame.move.x - off_x => x.value
      frame.move.y - off_y => y.value
    }
    idle->dragging (press.activation)
    dragging->idle (frame.release)
  }
}