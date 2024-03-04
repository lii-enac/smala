/*
*  Simple Ivy Sender app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2017-2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*  Contributors:
*    Jérémie Garcia    <jeremie.garcia@enac.fr>
*    Mathieu Poirier   <mathieu.poirier@enac.fr>
*
* note: 
* you can use "ivyprobe" to send a message to this application : helloIvy (.*)
* 
* note: on Mac 127.0.0.1 do not work correctly --> use 224.1.2.3 for test instead 
*/

use core
use base
use display
use gui
use comms

_action_
hook_action_on_die (Process src, Process data)
{   
    print ("\n\n helloIvy:  HAHHHAAA I don't WAANNNT to DIE !!!! \n\n")
}


_main_
Component root {
    Frame f ("helloIvy", 100, 100, 200, 200)
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
    Text txt (90, 70, "hello")

    /*------ Text to receive a Message ---*/
    FillColor rectFill2 (100, 100, 100)
    Rectangle receiver (50, 110, 100, 40, 5, 5)
    FillColor textFill2 (200, 200, 200)
    Text txt2 (90, 130, "...")

    /* ------- Log Printer to receive a Message in Terminal ---*/
    LogPrinter lp ("ivybus: ")

    IvyAccess ivybus ("224.1.2.3:2010", "helloIvy", "READY")
    {
        // define your regexs 
        // better to use (\\S*) than (.*) eq: "pos=(\\S*) alt=(\\S*)"
        String regex ("helloIvy (.*)")
        //...
    }

    //creating a connector to display incomming messages in the text
    ivybus.in.regex.[1] => txt2.text
    ivybus.in.regex.[1] => lp.input
   
    FSM pressFSM {
        State idle{
            100 =: rectFill.g
        }
        State pressed {
            150 =: rectFill.g
            // sending messages
            "helloIvy Hello you !" =: ivybus.out
        }
        idle -> pressed (sender.press)
        pressed -> idle (sender.release)
    }

    /* --------- on die ------------ */
    
	NativeAction na_on_die (hook_action_on_die, root, 1)
	ivybus.die -> na_on_die
    na_on_die -> ex
    // rem: we could also directly call : ivybus.die -> ex to quit without calling the hook
}
