use core
use base
use display
use gui


/*
 * Computes a component drag.
 * Works whatever the transforms applied before the drag branch of the components tree
 * (right matrix product in an homography, so in the local coordinates system of the node after the homography).
 * How it works:
 * - An Homography (passed as transforms argument) must be placed just before the component to drag.
 *   Translations will be accumulated into this homography (as well as any other transform).
 * - If the component to drag is a shape, it possesses a transform matrix, so it can directly be used.
 * - If the component to drag is not a shape, a shape must be placed with the component after 
 *   the homography. For example it can be a transparent Rectangle of the same size and position
 *   as the component to drag in order to have the same local coordinates system and to be able 
 *   to capture a press in the same zone (to trigger the drag interaction).
 */
_define_
Drag (Process shape, Process transforms) {

  FSM fsm {
    State idle

    State dragging {
        // Common API for mouse and single touch
        // to react only to mouse, use shape.mouse.[press|enter|move|leave|release]
      shape.move.local_x - shape.press.local_x =:> transforms.rightTranslateBy.dx
      shape.move.local_y - shape.press.local_y =:> transforms.rightTranslateBy.dy
    }
    
    idle -> dragging (shape.press)
    dragging -> idle (shape.release)
  }
}