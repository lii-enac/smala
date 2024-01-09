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

import Model
import View
import Controller

_main_
Component root {
    Frame f ("ivyReceiver", 100, 100, 600, 220)
    Exit ex (0, 1)
    f.close -> ex
    f.background_color.r = 50
    f.background_color.g = 50
    f.background_color.b = 50

    /*----- Variable -----*/
    List list_models
    List list_views
    List list_controlers
    Int reset_y (70)
    Int ty($reset_y)

    /*------ Button to clear ---*/
    Component btn_clear {
        Translation t (10, 10)
        FillColor rectFill (100, 100, 100)
        Rectangle rect (0, 0, 100, 40, 5, 5)
        FillColor textFill (200, 200, 200)
        Text txt (10, 20, "clear list")
    }

    /* ------- Log Printer to receive a Message in Terminal ---*/
    IvyAccess ivybus ("224.1.2.3:2010", "ivyReceiver", "READY")
    {
        String regex ("ivyModel i1=(.*) i2=(.*) i3=(.*) i4=(.*) i5=(.*) i6=(.*) i7=(.*)")
    }

    ivybus.in.regex.[1] -> receive_lambda : (root) {  // faire une lambda
        i =  find (root.ivybus)  // can't include in.regex because "in" or "regex" are not real children only the combination of in.regex.[x] is   
        Process m = Model (root.list_models, "", getString(i.in.regex.[1]), getString(i.in.regex.[2]), getString(i.in.regex.[3]), getString(i.in.regex.[4]), getString(i.in.regex.[5]), getString(i.in.regex.[6]), getString(i.in.regex.[7]))
        Process v = View (root.list_views, "", $root.ty)
        Controller (root.list_controlers, "", m, v)
        root.ty += 15
    }

    btn_clear.rect.release -> clear_lambda : (root) {
        for c : root.list_controlers {
            notify c.about_to_delete 
        }
        root.ty = root.reset_y
        
        //debug
        // dump root.list_controlers
        // dump root.list_views
        // dump root.list_models
    }

    FSM pressFSM {
        State idle{
            100 =: btn_clear.rectFill.g
        }
        State pressed {
            150 =: btn_clear.rectFill.g
        }
        idle -> pressed (btn_clear.rect.press)
        pressed -> idle (btn_clear.rect.release)
    }
}
