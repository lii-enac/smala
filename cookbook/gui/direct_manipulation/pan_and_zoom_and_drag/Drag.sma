use core
use base
use gui


import ScreenToLocal


/*
 * Computes a component drag in local coordinates system 
 * (independantly from the other transformations applied on the component).
 * Works by connecting interface members to a translation placed just before the component to drag.
 */
_define_
Drag (Component frame, Component toDrag) {

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
      ScreenToLocal scr2loc (toDrag)
      toDrag.press.x =: scr2loc.screenX
      toDrag.press.y =: scr2loc.screenY
      scr2loc.localX => pressX
      scr2loc.localY => pressY
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
      ScreenToLocal scr2loc (toDrag)
      frame.move.x => scr2loc.screenX
      frame.move.y => scr2loc.screenY
      tx0 + scr2loc.localX - pressX => tx
      ty0 + scr2loc.localY - pressY => ty
    }

    idle -> pressed (toDrag.press)
    pressed -> dragging (frame.move)
    pressed -> idle (frame.release)
    dragging -> idle (frame.release)
  }

}