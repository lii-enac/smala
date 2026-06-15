use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton
//import exec_env.main_loop


_define_
Counter_7GUIs ()
{
    Frame f("7GUIs Counter", 110, 0, 150, 100)                    // [7GUIs] The task is to build a frame containing...
    //f.close ->! mainloop

    FillColor _(#FFFFFF)                        // svg 'fill' defaults to black, which is hardly visible in smala lib's default dark theme
    Text T (10, 26, "")                         // [7GUIs] ...a label or read-only textfield T and...
    
    StandAlonePushButton B ("Count", 50, 10)    // [7GUIs] ...a button B. // FIXME label parameter should be after x and y, and it should be a "Button"

    // solution #1
    Int counter(0)                              // [7GUIs] Initially, the value in T is “0”...
    counter =:> T.text
    B.click -> {                                // [7GUIs] ...and each click of B...
        counter + 1 =: counter                  // [7GUIs] ...increases the value in T by one.
    }

    // // solution #2
    // Incr counter(0)                        // [7GUIs] Initially, the value in T is “0”...
    // B.click -> counter                     // [7GUIs] ...and each click of B increases...
    // counter.state =:> T.text               // [7GUIs] ...the value in T by one.
}
