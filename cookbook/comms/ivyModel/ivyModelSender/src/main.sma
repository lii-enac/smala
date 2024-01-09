/*
*  Simple Ivy Sender app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*  Contributors:
*    Mathieu Poirier   <mathieu.poirier@enac.fr>
*
* note: 
* you can use "ivyprobe" to send a message to this application : ivyModel (.*)
* 
* note: on Mac 127.0.0.1 do not work correctly --> use 224.1.2.3 for test instead 
*/

use core
use base
use display
use gui
use comms

_main_
Component root {
    Frame f ("IvyModelSender", 800, 100, 200, 220)
    Exit ex (0, 1)
    f.close -> ex
    f.background_color.r = 50
    f.background_color.g = 50
    f.background_color.b = 50

    Int nb_msg (10)

    /*------ Button to send a Message ---*/
    Component btn_send {
        Translation t (10, 150)
        FillColor rectFill (100, 100, 100)
        Rectangle sender (0, 0, 100, 40, 5, 5)
        FillColor textFill (200, 200, 200)
        Text txt (10, 20, "send")
    }

    /* ------- Log Printer to receive a Message in Terminal ---*/
    LogPrinter lp ("ivybus: ")

    IvyAccess ivybus ("224.1.2.3:2010", "ivyModelSender", "READY")
    {
    }

    btn_send.sender.release -> send_lambda : (root) {
        for (int i = 0; i < $root.nb_msg; i++) { 
            root.ivybus.out = "ivyModel i1=info1" + to_string(i) + " i2=info2" + to_string(i) + " i3=info3" + to_string(i) + " i4=info4" + to_string(i) + " i5=info5" + to_string(i) + " i6=info6" + to_string(i) + " i7=info7" + to_string(i)  
        } 
    }

    FSM pressFSM {
        State idle{
            100 =: btn_send.rectFill.g
        }
        State pressed {
            150 =: btn_send.rectFill.g
        }
        idle -> pressed (btn_send.sender.press)
        pressed -> idle (btn_send.sender.release)
    }
}