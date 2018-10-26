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

  Double pressX (0)
  Double pressY (0)

  FSM dragFsm {
    State idle

    State pressed {
      // Memorize press position in local coordinates
      ScreenToLocal s2l (localRef)
      localRef.press.x =: s2l.inX
      localRef.press.y =: s2l.inY
      s2l.outX => pressX
      s2l.outY => pressY
      // pressX/Y must be connected and not assigned as there is no guarantee that data flow
      // inside ScreenToLocal will be activated before the assignment on pressX/Y is done.
      // Using an AssignmentSequence here won't change anything.
    }

    State dragging {
      // Memorize initial translation
      Double tx0 (0)
      Double ty0 (0)
      tx =: tx0
      ty =: ty0

      // Compute added translation
      ScreenToLocal s2l (localRef)
      frame.move.x => s2l.inX
      frame.move.y => s2l.inY
      tx0 + s2l.outX - pressX => tx
      ty0 + s2l.outY - pressY => ty
    }

    idle -> pressed (localRef.press)
    pressed -> dragging (frame.move)
    pressed -> idle (frame.release)
    dragging -> idle (frame.release)
  }

}