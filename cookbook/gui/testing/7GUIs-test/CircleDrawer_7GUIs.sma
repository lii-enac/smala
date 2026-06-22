use core
use base
use display
use gui

import ../../7GUIs/6-circle-drawer/CircleDrawer

_define_
CircleDrawer_7GUIs () {
    Frame f ("7GUIs Circle Drawer", 0, 360, 500, 500)  // [7GUIs] The task is to build a frame containing...
    // f.close ->! mainloop
    mouseTracking = 1 // for enter and leave events FIXME should be automatic

    // Circle drawer with Undo/Redo
    CircleDrawer circle_drawer (f.width, f.height)

}