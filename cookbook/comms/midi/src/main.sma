use core
use base
use gui

import midi   // for self-contained midi message
import midi_p // for property-based midi message

import gui.widgets.Button

// try with https://tytel.org/helm/ or https://asb2m10.github.io/dexed/

_main_
Component root
{
    Frame f ("midi", 0, 0, 500, 500)

    Text _(10,20, "Press: note on - Release: note off - Drag X: pan - Drag Y: volume")

    Int channel (0)

    midi all_note_off ($channel, 0xB0, 0x7b, 0) // working
    midi all_sound_off ($channel, 0xB0, 0x78, 0) // working
    f.close -> all_note_off.do_it_2
    f.close -> all_sound_off.do_it_2

    Exit ex(0,1)
    f.close -> ex

    Int _val1(0) // with 1 arg message

    Component C3 {
        // noteon noteoff: working
        Rectangle r (50, 50, 50, 50)
        midi note_on ($channel, 0x90, 64, 90)
        midi note_off ($channel, 0x80, 64, 90)
        r.press -> note_on.do_it_2
        f.release -> note_off.do_it_2

        // control change: midi msg sent, and understood by midimonitor, but no change in synth ?!
        Int cc_midi_cmd(0xB0)

        Int cc_pan(0x0A) // works with apple internal synth
        //Int cc_pan(0x01) // modwheel: works with dexed
        Int pan(0)
        midi_p cc_pan_cmd (channel, cc_midi_cmd, cc_pan, pan)
        127*f.move.x/f.width => pan
        f.move.x -> cc_pan_cmd.do_it_2

        Int cc_vol(0x07)
        Int vol(127)
        midi_p cc_vol_cmd (channel, cc_midi_cmd, cc_vol, vol)
        127*f.move.y/f.height => vol
        f.move.y -> cc_vol_cmd.do_it_2

        Button btn_pc (f, "program change", 150, 50)
        //Rectangle r2 (150, 50, 50, 50)
        // program change: works with dexed, not with helm
        Int pc_midi_cmd(0xC0)
        Int pc(2)
        midi_p cc_pc_cmd (channel, pc_midi_cmd, pc, _val1)
        btn_pc.click -> cc_pc_cmd.do_it_1 // if commented, check bank select

        // bank select: works with dexed, not with helm
        Button btn_bs (f, "bank select", 300, 50)
        Int cc_bank_select_msb (0x0)
        Int bank_msb(1)
        midi_p bank_msb_cmd (channel, cc_midi_cmd, cc_bank_select_msb, bank_msb)
        Int cc_bank_select_lsb (0x20)
        Int bank_lsb(1)
        midi_p bank_lsb_cmd (channel, cc_midi_cmd, cc_bank_select_lsb, bank_lsb)
        // order-dependent !!
        btn_bs.click -> bank_msb_cmd.do_it_2
        btn_bs.click -> bank_lsb_cmd.do_it_2
        btn_bs.click -> cc_pc_cmd.do_it_1 // a bank select must be followed by a program change
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