use core
use base
use gui

import midi   // for self-contained message
import midi_p // for property-based message

// try with https://tytel.org/helm/ or https://asb2m10.github.io/dexed/

_main_
Component root
{
    Frame f ("midi", 0, 0, 500, 500)

    Text _(10,20, "Press: note on - Release: note off - Drag X: pan - Drag Y: volume")

    Int channel (0)

    midi all_note_off ($channel, 0xB0, 0x7b, 0) // working
    midi all_sound_off ($channel, 0xB0, 0x78, 0) // working
    f.close -> all_note_off.do_it
    f.close -> all_sound_off.do_it

    Exit ex(0,1)
    f.close -> ex

    Component C3 {
        // noteon noteoff: working
        Rectangle r (50, 50, 50, 50)
        midi note_on ($channel, 0x90, 64, 90)
        midi note_off ($channel, 0x80, 64, 90)
        r.press -> note_on.do_it
        f.release -> note_off.do_it

        // control change: midi msg sent, but no change in synth ??
        Int cc_midi_cmd(0xB0)

        Int cc_pan(0x0A)
        Int pan(0)
        Int cc_vol(0x07)
        Int vol(127)

        midi_p cc_pan_cmd (channel, cc_midi_cmd, cc_pan, pan)
        midi_p cc_vol_cmd (channel, cc_midi_cmd, cc_vol, vol)

        127*f.move.x/f.width => pan
        f.move.x -> cc_pan_cmd.do_it
        127*f.move.y/f.height => vol
        f.move.y -> cc_vol_cmd.do_it

        // bank select: not working...
        Rectangle r2 (150, 50, 50, 50)
        //Int cc_bank_select(0x20)
        //Int bank(1)
        //midi_p cc_bank_cmd (channel, cc_midi_cmd, cc_bank, bank)
        //r2.release -> cc_bank_cmd.do_it
        Int cc_program_change(0x00)
        Int pc(1)
        midi_p cc_pc_cmd (channel, cc_midi_cmd, cc_program_change, pc)
        r2.release -> cc_pc_cmd.do_it
    }
}

/*
        FSM note {
            State note_off {

            }
            State note_on {

            }
            note_off -> note_on (r.press, ../note_on.do_it)
            note_on -> note_off (f.release, ../note_off.do_it)
        }
*/  