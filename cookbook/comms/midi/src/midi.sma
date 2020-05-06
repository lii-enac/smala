use core
use base

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
    std::cerr << midiout->getPortName () << std::endl;
    // Open first available port.
    midiout->openPort( 0 );
}

void
send_midi(int channel, int command, int val1, int val2)
{
    init_midi_if_not_inited ();

    std::vector<unsigned char> message(3);
    message[0] = command | channel;
    message[1] = val1;
    message[2] = val2;
    midiout->sendMessage( &message );
    //std::cout << hex << channel << " " << command << " " << val1 << " " << val2 << dec << std::endl;
}

void
send_midi(int channel, int command, int val1)
{
    init_midi_if_not_inited ();

    std::vector<unsigned char> message(2);
    message[0] = command | channel;
    message[1] = val1;
    midiout->sendMessage( &message );
    //std::cout << hex << channel << " " << command << " " << val1 << dec << std::endl;
}

void
midi_cmd_2(Process* p)
{
    Process *data = reinterpret_cast<Process*> (get_native_user_data (p));
    auto channel = dynamic_cast<IntProperty*> (data->find_child("channel"))->get_value ();
    auto cmd = dynamic_cast<IntProperty*> (data->find_child("command"))->get_value ();
    auto val1 = dynamic_cast<IntProperty*> (data->find_child("val1"))->get_value ();
    auto val2 = dynamic_cast<IntProperty*> (data->find_child("val2"))->get_value ();

    send_midi (channel, cmd, val1, val2);
}

void
midi_cmd_1(Process* p)
{
    Process *data = reinterpret_cast<Process*> (get_native_user_data (p));
    auto channel = dynamic_cast<IntProperty*> (data->find_child("channel"))->get_value ();
    auto cmd = dynamic_cast<IntProperty*> (data->find_child("command"))->get_value ();
    auto val1 = dynamic_cast<IntProperty*> (data->find_child("val1"))->get_value ();
   
    send_midi (channel, cmd, val1);
}


%}

_define_
midi (int chan, int cmd, int v1, int v2)
{
    Int channel (chan)
    Int command (cmd)
    Int val1 (v1)
    Int val2 (v2)

    NativeAction midi_cmd_na_2 (midi_cmd_2, this, 1)
    Spike do_it_2
    do_it_2 -> midi_cmd_na_2

    NativeAction midi_cmd_na_1 (midi_cmd_2, this, 1)
    Spike do_it_1
    do_it_1 -> midi_cmd_na_1
}

/*
https://ccrma.stanford.edu/~craig/articles/linuxmidi/misc/essenmidi.html

Command 	Meaning 	       # parameters param 1 	 param 2
0x80 	    Note-off 	            2 	    key 	     velocity
0x90 	    Note-on 	            2 	    key 	     velocity
0xA0 	    Aftertouch 	            2 	    key 	     touch
0xB0 	    Continuous controller 	2 	    controller # controller value
0xC0 	    Patch change 	        2?? 	instrument # 	
0xD0 	    Channel Pressure 	    1 	    pressure
0xE0 	    Pitch bend 	            2 	    lsb (7 bits) msb (7 bits)
0xF0 	    (non-musical commands) 	

Midi GM2 Bank Select:
0xB0 (CC)   Bank select             2        0           bank MSB
0xB0 (CC)   Bank select             2        32          bank LSB
0xC0 	    Patch change 	        2?? 	 instrument # 	


command 	meaning 	                                # param
0xF0 	    start of system exclusive message 	        variable
0xF1 	    MIDI Time Code Quarter Frame (Sys Common)	
0xF2 	    Song Position Pointer (Sys Common)	
0xF3 	    Song Select (Sys Common) 	
0xF4 	    ??? 	
0xF5 	    ??? 	
0xF6 	    Tune Request (Sys Common) 	
0xF7 	    end of system exclusive message 	        0
0xF8 	    Timing Clock (Sys Realtime) 	
0xFA 	    Start (Sys Realtime) 	
0xFB 	    Continue (Sys Realtime) 	
0xFC 	    Stop (Sys Realtime) 	
0xFD 	    ??? 	
0xFE 	    Active Sensing (Sys Realtime) 	
0xFF

Control Change:
https://www.midi.org/specifications-old/item/table-3-control-change-messages-data-bytes-2

0x78: all sound off
0x7B: all notes off

*/
