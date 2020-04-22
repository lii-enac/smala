use core
use base
use display
use gui

import dnd

_main_
Component root
{
    Frame f ("my frame", 0, 0, 1000, 1000)

	OutlineWidth ow(10)
	FillColor fc(255,0,0)
    OutlineColor _(0,0,255)

    Component trash {
        Rectangle icon(500,500,30,100)
    }
  
    Translation t(0,0)
    Circle mobile(100, 100, 40)

    dnd _(f, t, mobile, trash)
}
