// [7GUIs] Temperature Converter
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#temp
// [7GUIs] Challenges: bidirectional data flow, user-provided text input.

use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton
import gui.widgets.UITextField
import gui.widgets.HBox
import gui.widgets.Label


_main_
Component root {
    Frame f ("7GUIs Temperature Converter UNFINISHED", 0, 0, 600, 600)  // [7GUIs] The task is to build a frame containing
    f.close ->! mainloop

    UITextField _TC              // [7GUIs] two textfields TC and TF representing the temperature in Celsius and Fahrenheit, respectively.
    Label c_text(" celsius =")   // [7GUIs] (implicit) and...
    Label f_text(" farenheit")   // [7GUIs] ... two labels
    UITextField _TF              // [7GUIs] Initially, both TC and TF are empty.

    HBox h(f) {}
    addChildrenTo h.items {
        _TC,
        c_text,
        _TF,
        f_text
    }
    // FIXME now that all widgets have been reparented by addChildrenTo, they are inaccessible :-/
    TC aka h.items.[1]
    TF aka h.items.[3]

    TC.text_color = #FFFFFF
    TF.text_color = #FFFFFF


    Double C(0)
    Double F(0)

    // The code below could be the most straightforward but it introduces a cycle
    // (F - 32) * (5/9.) => C        // [7GUIs] C = (F - 32) * (5/9)
    //  C * (9/5.) + 32 => F         // [7GUIs] F = C * (9/5) + 32
    // It would be better done this way, but more work needs to be done in the execution engine FIXME
    
    // To avoid the cycle, we use a Switch to specify which direction is required according to the user interaction
    Switch conv_dir (from_C_to_F) {  // [7GUIs] The formula for converting a temperature C in Celsius into a temperature F in Fahrenheit is
        Component from_F_to_C {
            (F - 32) * (5/9.) =:> C  // [7GUIs] C = (F - 32) * (5/9)
        }
        Component from_C_to_F {      // [7GUIs] and the dual direction is
            C * (9/5.) + 32 =:> F    // [7GUIs] F = C * (9/5) + 32.
        }
    }

    TC.text -> {                        // [7GUIs] When the user enters a numerical value into TC
        "from_C_to_F" =: conv_dir.state // [7GUIs] the corresponding value in TF is automatically updated
    }
    TF.text -> {                        // [7GUIs] and
        "from_F_to_C" =: conv_dir.state // [7GUIs] vice versa.
    }

    // [7GUIs] TODO When the user enters a non-numerical string into TC the value in TF is not updated and vice versa.

    TC.text =:> C
    toString(C) => TC.field.content.text

    TF.text =:> F
    toString(F) => TF.field.content.text

    // debug
    // TextPrinter tp
    // "" + C + " " + F + " " + TF.text + " " + conv_dir.state =:> tp.input
}



// [7GUIs] The task is to build a frame containing two textfields TC and TF representing the temperature in Celsius and Fahrenheit, respectively.
// [7GUIs] Initially, both TC and TF are empty.
// [7GUIs] When the user enters a numerical value into TC the corresponding value in TF is automatically updated and vice versa.
// [7GUIs] When the user enters a non-numerical string into TC the value in TF is not updated and vice versa.
// [7GUIs] The formula for converting a temperature C in Celsius into a temperature F in Fahrenheit is C = (F - 32) * (5/9)
// [7GUIs] and the dual direction is F = C * (9/5) + 32.  

// [7GUIs] Temperature Converter increases the complexity of Counter
// [7GUIs] by having bidirectional data flow between the Celsius and Fahrenheit inputs
// [7GUIs] and the need to check the user input for validity.

// [7GUIs] A good solution will make the bidirectional dependency very clear with minimal boilerplate code.

// [7GUIs] Temperature Converter is inspired by the Celsius/Fahrenheit converter from the book Programming in Scala.
// ([SCO]  also a famous example from Fabrik)
// [7GUIs] It is such a widespread example—sometimes also in the form of a currency converter—that one could give a thousand references.
// ([SCO]  inc. those of Jef Raskin)
// [7GUIs] The same is true for the Counter task.
