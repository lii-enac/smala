use core 
use base
use gui

import Chat

_main_
Component root {
    Frame f ("Chat", 500, 500, 400, 500)
    Exit ex (0, 1)
    f.close->ex
    mouseTracking = 1
    f.background_color.r = 240
    f.background_color.g = 240
    f.background_color.b = 240
    Chat chat (0, 0, 500, 500, f)
    //_DEBUG_GRAPH_CYCLE_DETECT = 1
}