use core
use base
use gui


_define_
CircleItem (Process root_, int x, int y)
{
    FillColor fc(200,200,200)               // [7GUIs] (the color gray)
    NoFill nf                               // [7GUIs] ...an unfilled...
    PickFill _
    OutlineColor oc(255,255,255)
    Circle c(x, y, 10)                      // [7GUIs] ...circle with a fixed diameter whose center is the left-clicked point.

    c.enter ->! nf                          // [7GUIs] The circle nearest to the mouse pointer such that the distance from its center to the pointer is less than its radius, if it exists, is filled with the color gray.
    c.leave -> nf

    root aka root_

    // FIXME should be right press 
    c.press -> {                            // [7GUIs] Right-clicking C will make a popup menu appear with one entry “Adjust diameter..”.
        c.press.x =: root.menu.x
        c.press.y =: root.menu.y
    }

    c.press -> (this) {
        setRef(this.root.gcircle_ref_, this)
    }
    // c.leave -> (this) {
    //     setRef(this.root.gcircle_ref_, null)
    // }
}