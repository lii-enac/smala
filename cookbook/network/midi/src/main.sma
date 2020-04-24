use core
use base
use gui

import midi

_main_
Component root
{
    Frame f ("midi", 0, 0, 500, 500)

    // try with https://tytel.org/helm/ or https://asb2m10.github.io/dexed/
    midi m
    Rectangle r(50,50,50,50)
    r.press -> m.note_on
}