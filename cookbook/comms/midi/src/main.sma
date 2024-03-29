use core
use base
use gui

import midi   // for self-contained midi message
import midi_p // for property-based midi message

_native_code_
%{
    extern void clear_midi ();
%}

import gui.widgets.PushButton

// try with https://tytel.org/helm/ or https://asb2m10.github.io/dexed/

_main_
Component root
{
    Frame f ("midi", 0, 0, 500, 500)
    Exit ex(0,1)
    f.close -> (root) {
        clear_midi ()
    }
    f.close -> ex


    Text _(10,20, "Press: note on - Release: note off - Drag X: pan - Drag Y: volume")

    Int channel (0)

    midi all_note_off ($channel, 0xB0, 0x7b, 0) // working
    midi all_sound_off ($channel, 0xB0, 0x78, 0) // working
    f.close -> all_note_off.do_it_2
    f.close -> all_sound_off.do_it_2



    Int _val1(0) // with 1 arg message

    // -------
    // examples with midi message with ints
    Component C4 {    
        // noteon noteoff: working
        Rectangle r (50, 50, 50, 50)
        midi note_on ($channel, 0x90, 60, 90) // 60 = C4
        midi note_off ($channel, 0x80, 60, 90)
        r.press -> note_on.do_it_2
        f.release -> note_off.do_it_2
    }

    // -------
    // examples with midi_p message with IntProperty

    // control change
    Int cc_midi_cmd(0xB0)

    Int cc_pan(0x0A) // works with apple internal synth, unsupported in dexed and helm
    //Int cc_pan(0x01) // modwheel: works with dexed
    Int pan(0)
    midi_p cc_pan_cmd (channel, cc_midi_cmd, cc_pan, pan)
    127*f.move.x/f.width => pan
    f.move.x -> cc_pan_cmd.do_it_2

    Int cc_vol(0x07) // works with apple internal synth, unsupported in dexed and helm
    Int vol(127)
    midi_p cc_vol_cmd (channel, cc_midi_cmd, cc_vol, vol)
    127*f.move.y/f.height => vol
    f.move.y -> cc_vol_cmd.do_it_2

    // program change: works with dexed, not with helm
    PushButton btn_pc (f, "program change", 150, 50)
    Int pc_midi_cmd(0xC0)
    Int pc(2)
    midi_p cc_pc_cmd (channel, pc_midi_cmd, pc, _val1)
    btn_pc.click -> cc_pc_cmd.do_it_1

    // bank select: works with dexed, not with helm
    PushButton btn_bs (f, "bank select", 300, 50)
    Int cc_bank_select_msb (0x0)
    Int bank_msb(1)
    midi_p bank_msb_cmd (channel, cc_midi_cmd, cc_bank_select_msb, bank_msb)
    Int cc_bank_select_lsb (0x20)
    Int bank_lsb(1)
    midi_p bank_lsb_cmd (channel, cc_midi_cmd, cc_bank_select_lsb, bank_lsb)
    // order-dependent !!
    btn_bs.click -> bank_msb_cmd.do_it_2
    bank_msb_cmd.do_it_2 -> bank_lsb_cmd.do_it_2
    bank_lsb_cmd.do_it_2 -> cc_pc_cmd.do_it_1

    /* ideal syntax :
       btn_bs.click
    -> bank_msb_cmd.do_it_2
    -> bank_lsb_cmd.do_it_2
    -> cc_pc_cmd.do_it_1

    voire mixer des assignments:
       btn_bs.click
    -> bank_msb_cmd.do_it_2
    -> bank_lsb_cmd.do_it_2
    -> toInt(edit_box.value) =: _val1
    -> cc_pc_cmd.do_it_1
    */

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