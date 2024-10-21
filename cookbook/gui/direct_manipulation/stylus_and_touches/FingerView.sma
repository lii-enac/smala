use core
use base
use gui

_define_
FingerView (Process touch) {
    Translation tr (0, 0)

    Circle finger (0, 0, 50)

    touch.move.x =:> tr.tx // finger.cx
    touch.move.y =:> tr.ty // finger.cy

    FillColor _ (#FFFFFF)
    Text t (-20, -70, toString(touch.id))
}
