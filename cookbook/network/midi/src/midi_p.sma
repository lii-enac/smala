use core
use base

_native_code_
%{
extern void midi_cmd_2(Process* p);
%}

_define_
midi_p (Process chan, Process cmd, Process v1, Process v2)
{
    Int channel (0)
    Int command (0)
    Int val1 (0)
    Int val2 (0)

    chan =:> channel
    cmd =:> command
    v1 =:> val1
    v2 =:> val2

    NativeAction midi_cmd_na (midi_cmd_2, this, 1)
    Spike do_it
    do_it -> midi_cmd_na
}

// attic
/*    Int a(0)
    NativeAction cpp_na (cpp_action, this, 1)

    Int command(0x90)
    Int channel(0)
    Int note (64)
    Int velocity (90)

    Int val0(0)
    Int val1(0)
    Int val2(0)

    AssignmentSequence set_note_on_values (1) {
        channel =: val0
        note =: val1
        velocity =: val2
    }

    NativeAction note_on_na (note_on, this, 1)
    Spike note_on
    note_on->set_note_on_values
    set_note_on_values->note_on_na

    TextPrinter tp
    val1 => tp.input
*/
