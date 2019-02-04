use core
use base
use gui


/*
 * Computes a component drag in localRef coordinates system 
 * (independantly from the other transformations applied before on the component).
 * How it works:
 * - If the component to drag is a shape, it possesses a transform matrix,
 *   so it can directly be used as localRef.
 * - If the component to drag is not a shape, a shape must be placed with the component after 
 *   the Translation instance. For example it can be a transparent Rectangle of the same size 
 *   and position as the component to drag in order to have the same local coordinates system
 *   and to be able to capture a press in the same zone (to trigger the drag interaction).
 * - A translation must be placed just before the component to drag [and the local reference].
 *   It's members tx/ty must be fed by a Connector from interface members.
 */
_define_
Drag (Component frame, Component localRef) {

  /* ----- interface ----- */
  Double tx (0)
  Double ty (0)
  /* --- end interface --- */

  FSM dragFsm {
    State idle

    State pressed {
      // Memorize press position in local coordinates
      ScreenToLocal s2l (localRef)
      localRef.press.x =: s2l.inX
      localRef.press.y =: s2l.inY
    }

    State dragging {
      // Memorize initial translation
      AdderAccumulator atx (0, 0, 0)
      tx :: atx.input
      AdderAccumulator aty (0, 0, 0)
      ty :: aty.input

      // Convert mouse move to local coordinates
      ScreenToLocal s2l (localRef)
      frame.move.x => s2l.inX
      frame.move.y => s2l.inY

      // Compute added translation
      s2l.outX - pressed.s2l.outX => atx.input
      s2l.outY - pressed.s2l.outY => aty.input
      atx.result => tx
      aty.result => ty
    }

    idle -> pressed (localRef.press)
    pressed -> dragging (frame.move)
    pressed -> idle (frame.release)
    dragging -> idle (frame.release)
  }

  // MORE COMPACT VERSION (2 states FSM + press outside)
  /*
  // Convert mouse press to local coordinates
  ScreenToLocal s2l_press (localRef)
  localRef.press.x => s2l_press.inX
  localRef.press.y => s2l_press.inY

  FSM dragFsm {
    State idle

    State dragging {
      // Memorize initial translation
      AdderAccumulator atx (0, 0, 0)
      tx :: atx.input
      AdderAccumulator aty (0, 0, 0)
      ty :: aty.input

      // Convert mouse move to local coordinates
      ScreenToLocal s2l_move (localRef)
      frame.move.x => s2l_move.inX
      frame.move.y => s2l_move.inY

      // Compute added translation
      s2l_move.outX - s2l_press.outX => atx.input
      s2l_move.outY - s2l_press.outY => aty.input
      atx.result => tx
      aty.result => ty
    }

    idle -> dragging (localRef.press)
    dragging -> idle (frame.release)
  }
  */
}