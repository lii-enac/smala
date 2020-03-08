use core
use exec_env
use base
use display
use gui

import TouchData

/*
 * Computes a component drag in multitouch.
 * Works whatever the transforms applied before the drag branch of the components tree
 * (right matrix product in an homography, so in the local coordinates system of the node after the homography).
 * How it works:
 * - An Homography (passed as transforms argument) must be placed just before the component to drag [and the local ref].
 *   Translations will be accumulated into this homography (as well as any other transform).
 * - If the component to drag is a shape, it possesses a transform matrix, so it can directly be used.
 * - If the component to drag is not a shape, a shape must be placed with the component after 
 *   the homography. For example it can be a transparent Rectangle of the same size and position
 *   as the component to drag in order to have the same local coordinates system and to be able 
 *   to capture a press in the same zone (to trigger the drag interaction).
 */
_define_
TouchDrag (Process _frame, Process _shape, Process transforms) {

  frame aka _frame // alias to access frame from natives
  shape aka _shape // alias to access shape from natives

  Dictionary d_touch // TODO en faire un objet global
  TouchData activeTouchData

  // monitor touch presses on shape to get reference of new touches on shape only
  shape.touches.$added -> (this) {
    if (this.activeTouchData.touchId == 0) {
      t = getRef (&this.shape.touches.$added)
      this.activeTouchData.touchId = t.id
      this.activeTouchData.press.x = t.x
      this.activeTouchData.press.y = t.y
  
      addChildrenTo this {
        Component touchEvents {
          t.x =:> this.activeTouchData.move.x
          t.y =:> this.activeTouchData.move.y
          t.y -> this.activeTouchData.moved
        }
        setRef (&this.d_touch.key, &t)
        setRef (&this.d_touch.value, &touchEvents)
        run this.d_touch.add
      }
  
      run this.activeTouchData.pressed
    }
  }

  // monitor touch releases on frame and not on shape
  // otherwise if the touches move outside shape, the touch is considered released
  frame.touches.$removed -> (this) {
    t = getRef (&this.frame.touches.$removed)
    // interrupt drag only if the released touch is the active one on the shape
    if (t.id == this.activeTouchData.touchId) {
      setRef (&this.d_touch.key, &t)
      p = getRef (&this.d_touch.value)
      run this.d_touch.delete
      delete p
      this.activeTouchData.touchId = 0
      run this.activeTouchData.released
    }
  }

  // TODO SEPARER LE COMPOSANT EN 2 ?

  FSM fsm {
    State idle

    State pressed {
      ScreenToLocal s2l (shape)
      activeTouchData.press.x =:> s2l.inX
      activeTouchData.press.y =:> s2l.inY
    }

    State dragging {
      ScreenToLocal s2l (shape)
      activeTouchData.move.x =:> s2l.inX
      activeTouchData.move.y =:> s2l.inY
      s2l.outX - pressed.s2l.outX =:> transforms.rightTranslateBy.dx
      s2l.outY - pressed.s2l.outY =:> transforms.rightTranslateBy.dy
    }
    
    idle -> pressed (activeTouchData.pressed)
    pressed -> idle (activeTouchData.released)
    pressed -> dragging (activeTouchData.moved)
    dragging -> idle (activeTouchData.released)
    // TODO AFFINER FSM ET/OU REDIRECTION POUR N'INTERROMPRE LE DRAG QUE SUR LE DERNIER RELEASE
  }

}