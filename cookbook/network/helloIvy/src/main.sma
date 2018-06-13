/*
*  Simple Ivy Sender app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*  Contributors:
*    Jérémie Garcia    <jeremie.garcia@enac.fr>
*
* note: 
* you can use "ivyprobe" to send a message to this application : smala (.*)
*/

use core
use base
use gui
use comms

_main_
Component root {
    Frame f ("Hello Ivy", 100, 100, 200, 200)
    Exit ex (0, 1)
    f.close -> ex

    /*----- Background -----*/
    FillColor bgFill (50,50,50)
    Rectangle bgRect (0, 0, 0, 0 , 0, 0)
    f.width => bgRect.width
    f.height => bgRect.height

    /*------ Button to send a Message ---*/
    FillColor rectFill (100, 100, 100)
    Rectangle sender (50, 50, 100, 40, 5, 5)
    FillColor textFill (200, 200, 200)
    Text txt (90, 70, "hello")

    /*------ Text to receive a Message ---*/
    FillColor rectFill2 (100, 100, 100)
    Rectangle receiver (50, 110, 100, 40, 5, 5)
    FillColor textFill2 (200, 200, 200)
    Text txt2 (90, 130, "...")

    IvyAccess ivybus ("127.0.0.1:2010", "smala", "READY")

    //creating a connector to display incomming messages in the text
    Connector receiverBus (ivybus, "in/smala (.*)/1", txt2, "text")

    FSM pressFSM {
        State idle{
            100 =: rectFill.g
        }
        State pressed {
            150 =: rectFill.g
            "smala Hello" =: ivybus.out
        }
        idle -> pressed (sender.press)
        pressed -> idle (sender.release)
    }
}
run root
run syshook
