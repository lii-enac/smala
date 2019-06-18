use core
use base
use gui


/*
 * Computes a component drag.
 * Works whatever the transforms applied before the drag branch of the components tree
 * (right matrix product in an homography, so in the local coordinates system of the node after the homography).
 * How it works:
 * - An Homography (passed as transforms argument) must be placed just before the component to drag [and the local ref].
 *   Translations will be accumulated into this homography (as well as any other transform).
 * - If the component to drag is a shape, it possesses a transform matrix,
 *   so it can directly be used as localRef.
 * - If the component to drag is not a shape, a shape must be placed with the component after 
 *   the homography. For example it can be a transparent Rectangle of the same size and position
 *   as the component to drag in order to have the same local coordinates system and to be able 
 *   to capture a press in the same zone (to trigger the drag interaction).
 */
_define_
Drag (Component frame, Component localRef, Component transforms) {

  FSM fsm {
    State idle

    State pressed {
      ScreenToLocal s2l (localRef)
      frame.press.x =:> s2l.inX
      frame.press.y =:> s2l.inY
    }

    State dragging {
      ScreenToLocal s2l (localRef)
      frame.move.x =:> s2l.inX
      frame.move.y =:> s2l.inY
      s2l.outX - pressed.s2l.outX =:> transforms.rightTranslateBy.dx
      s2l.outY - pressed.s2l.outY =:> transforms.rightTranslateBy.dy
    }
    
    idle -> pressed (localRef.press)
    pressed -> idle (frame.release)
    pressed -> dragging (frame.move)
    dragging -> idle (frame.release)
  }

  // MORE COMPACT VERSION (press outside, only 2 states FSM)
  /*
  ScreenToLocal s2l_press (localRef)
  localRef.press.x =:> s2l_press.inX
  localRef.press.y =:> s2l_press.inY

  FSM fsm {
    State idle
    State dragging {
      ScreenToLocal s2l_move (localRef)
      frame.move.x =:> s2l_move.inX
      frame.move.y =:> s2l_move.inY
      s2l_move.outX - s2l_press.outX =:> transforms.rightTranslateBy.dx
      s2l_move.outY - s2l_press.outY =:> transforms.rightTranslateBy.dy
    }
    idle -> dragging (localRef.press)
    dragging -> idle (frame.release)
  }
  */
}