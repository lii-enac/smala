use core
use base
use gui

import midi

// try with https://tytel.org/helm/ or https://asb2m10.github.io/dexed/

_main_
Component root
{
    Frame f ("midi", 0, 0, 500, 500)
    Exit ex(0,1)

    midi all_note_off (123, 0, 0, 0) // doesn't seem to work with helm...
    f.close -> all_note_off.do_it

    f.close -> ex

    Component C3 {
        Rectangle r (50, 50, 50, 50)
        midi note_on (0x90, 0, 64, 90)
        midi note_off (0x80, 0, 64, 90)
        r.press -> note_on.do_it
        f.release -> note_off.do_it

        /*Int pan(0)
        midi cc (0xB0, 0, 0x0A, $pan)
        127*f.move.x/f.width => pan
        f.move -> cc.do_it*/
    }
}