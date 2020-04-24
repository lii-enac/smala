use core

_native_code_
%{
#include <unistd.h>
#include "RtMidi.h"

RtMidiOut *midiout = nullptr;

void
init_midi_if_not_inited ()
{
    if (midiout) return;

    midiout = new RtMidiOut();

    // Check available ports.
    unsigned int nPorts = midiout->getPortCount();
    if ( nPorts == 0 ) {
        std::cout << "No ports available!\n";
        return;
    }
    // Open first available port.
    midiout->openPort( 0 );
}

void
send_midi(int command, int channel, int val1, int val2)
{
    std::vector<unsigned char> message(3);
    message[0] = command | channel;
    message[1] = val1;
    message[2] = val2;
    midiout->sendMessage( &message );
}

%}


_action_
note_on (Process p)
%{
    init_midi_if_not_inited ();

    Process *data = reinterpret_cas<Process*> (get_native_user_data (p));
    auto val0 = dynamic_cast<IntProperty*> (data->find_child("val0"))->get_value ();
    auto val1 = dynamic_cast<IntProperty*> (data->find_child("val1"))->get_value ();
    auto val2 = dynamic_cast<IntProperty*> (data->find_child("val2"))->get_value ();

    std::cout << "note on " << val0 << " " << val1 << " " << val2 << std::endl;
    send_midi (0x90, val0, val1, val2);
%}

_action_
cpp_action (Process p)
%{
    init_midi_if_not_inited ();

    std::vector<unsigned char> message;
    // Send out a series of MIDI messages.

    unsigned int command = 0;
    unsigned int channel = 1;

    // Program change: 192, 5 (0xc0+channel)
    //message.push_back( 192 );
    message.push_back( 0xc0 + channel );
    message.push_back( 5 );
    midiout->sendMessage( &message );

    // Control Change: 176, 7, 100 (volume)
    message[0] = 176;
    message[1] = 7;
    message.push_back( 100 );
    midiout->sendMessage( &message );

    // Note On: 144, 64, 90
    message[0] = 144;
    message[1] = 64;
    message[2] = 90;
    midiout->sendMessage( &message );
    std::cout << "note on" << std::endl;

    //SLEEP( 500 ); // Platform-dependent ... see example in tests directory.
    usleep(500000);

    // Note Off: 128, 64, 40
    message[0] = 128;
    message[1] = 64;
    message[2] = 0;
    midiout->sendMessage( &message );

    std::cout << "note off" << std::endl;
%}

_define_
midi ()
{
    Int a(0)
    NativeAction cpp_na (cpp_action, this, 1)

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
    note_on -> note_on_na
    //note_on->set_note_on_values
    //set_note_on_values->note_on_na

}
