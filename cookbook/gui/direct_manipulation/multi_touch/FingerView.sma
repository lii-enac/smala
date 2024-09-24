use core
use base
use gui

_define_
FingerView (Process touch) {
    Circle finger (-100, -100, 100)
    touch.move.x =:> finger.cx
    touch.move.y =:> finger.cy
}
