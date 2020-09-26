use core
use base
use display
use gui

_main_
Component root
{
    Frame f ("my frame", 0, 0, 800, 800)
    Exit ex (0, 1)
    f.close -> ex
    mouseTracking = 1

    LinearGradient _(0,0,1,0, DJN_PAD_FILL, DJN_LOCAL_COORDS) {
        GradientStop _(0,0,0,1,0)
        GradientStop _(155,155,155,1,1)
    }
    Rectangle bg (0,0, 00,100)
    f.width =:> bg.width
    f.height =:> bg.height

    FillColor _ (100,100,100)

    Rectangle left (50,50, 100,100)
    Rectangle right (600,50, 100,100)

    Rectangle mobile (50,400, 100,100)
    f.move.x => mobile.x
    f.move.y => mobile.y
}