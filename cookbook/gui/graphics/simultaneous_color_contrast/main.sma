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
    //mouseTracking = 1

    FillColor _ (60,249,1)
    Rectangle bg_left (0,0, 0,100)
    f.width / 2 =:> bg_left.width
    f.height =:> bg_left.height

    FillColor _ (255,58,219)
    Rectangle bg_right (0,0, 0,100)
    f.width / 2 =:> bg_right.x, bg_right.width
    f.height =:> bg_right.height

    FillColor _ (0,133,255)
    Rectangle left_up (50,50, 100,100)
    FillColor _ (146,55,255)
    Rectangle left_down (50,350, 100,100)

    FillColor _ (146,55,255)
    Rectangle right (600,200, 100,100)
    f.width - 150 =:> right.x
}