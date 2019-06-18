use core
use base
use gui

_define_
Drag (Component frame, Component target, Component transforms) {
    FSM fsm {
        State idle
        State pressed {
            ScreenToLocal s2l (target)
            frame.press.x =:> s2l.inX
            frame.press.y =:> s2l.inY
        }
        State dragging {
            ScreenToLocal s2l (target)
            frame.move.x =:> s2l.inX
            frame.move.y =:> s2l.inY
            s2l.outX - pressed.s2l.outX =:> transforms.rightTranslateBy.dx
            s2l.outY - pressed.s2l.outY =:> transforms.rightTranslateBy.dy
        }
        idle -> pressed (target.press)
        pressed -> idle (frame.release)
        pressed -> dragging (frame.move)
        dragging -> idle (frame.release)
    }

}