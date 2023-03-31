use core 
use base
use gui

import Chat

_main_
Component root {
    Frame f ("Chat", 500, 0, 210, 310)
    Exit ex (0, 1)
    f.close->ex
    mouseTracking = 1 // for send button :-/

    f.background_color.r = 240
    f.background_color.g = 240
    f.background_color.b = 240

    Chat chat (0, 0, 200, 300, f)
    //f.{width, height} =:> chat.{width, height}

    _DEBUG_GRAPH_CYCLE_DETECT = 1
}