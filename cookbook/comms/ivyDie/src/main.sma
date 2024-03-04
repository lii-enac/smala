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
* you can use "ivyprobe" to send a message to this application : ivyDie (.*)
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
    Frame f ("ivyDie", 300, 100, 200, 200)
    Exit ex (0, 1)
    f.close -> ex

    /*----- Background -----*/
    FillColor bgFill (50,50,50)
    Rectangle bgRect (0, 0, 0, 0 , 0, 0)
    f.width =:> bgRect.width
    f.height =:> bgRect.height

    /*------ Button to send a Message ---*/
    FillColor rectFill (100, 100, 100)
    Rectangle sender (50, 50, 100, 40, 5, 5)
    FillColor textFill (200, 200, 200)
    Text txt (60, 70, "kill helloIvy")


    /* ------- Log Printer to receive a Message in Terminal ---*/
    LogPrinter lp ("ivybus: ")

    IvyAccess ivybus ("224.1.2.3:2010", "ivyDie", "READY")
    {
    }

   
    FSM pressFSM {
        State idle{
            100 =: rectFill.g
        }
        State pressed {
            150 =: rectFill.g
            // sending messages
            "helloIvy" =: ivybus.send_die
        }
        idle -> pressed (sender.press)
        pressed -> idle (sender.release)
    }
}
