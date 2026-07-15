/*
*  Smala cookbook 2-temp-converter
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2025-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
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
    Frame f ("7GUIs Temp Converter") // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop

    HBox h {
        UITextField TC               // [7GUIs] ...two textfields TC and TF representing the temperature in Celsius and Fahrenheit, respectively.
        Label LC(" Celsius =")       // [7GUIs] (implicit) and...
        UITextField TF               // [7GUIs] ... two labels
        Label LF(" Farenheit")       // [7GUIs] Initially, both TC and TF are empty.
    }

    Double C(0)
    Double F(0)

    // The code below is the most straightforward but it introduces a cycle
    // More work needs to be done in the execution engine to handle it seamlessly if it's ever possible
    // Meanwhile there is an _AUTHORIZE_CYCLE variable we can set, but it's global to the program...
    _AUTHORIZE_CYCLE = 1

    (F - 32) * (5/9.) => C           // [7GUIs] C = (F - 32) * (5/9)
      C * (9/5.) + 32 => F           // [7GUIs] F = C * (9/5) + 32
    
    
    // To avoid the cycle and not use _AUTHORIZE_CYCLE,
    // we can use a Switch to specify which dependency direction is required
    // according to the user interaction i.e. which field is changed
    // Switch conv_dir (from_C_to_F) {  // [7GUIs] The formula for converting a temperature C in Celsius into a temperature F in Fahrenheit is...
    //     Component from_F_to_C {
    //         (F - 32) * (5/9.) =:> C  // [7GUIs] C = (F - 32) * (5/9)...
    //     }
    //     Component from_C_to_F {      // [7GUIs] ...and the dual direction is...
    //         C * (9/5.) + 32 =:> F    // [7GUIs] F = C * (9/5) + 32.
    //     }
    // }

    // h.TC.text -> {                      // [7GUIs] When the user enters a numerical value into TC...
    //     "from_C_to_F" =: conv_dir.state // [7GUIs] ...the corresponding value in TF is automatically updated...
    // }
    // h.TF.text -> {                      // [7GUIs] ...and
    //     "from_F_to_C" =: conv_dir.state // [7GUIs] ...vice versa.
    // }

    string regex_str = "\\s*[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)\\s*" // real number regex, as per https://stackoverflow.com/a/12643073/2036022

    Regex regex_num_C (regex_str)
    h.TC.text =:> regex_num_C.input     // [7GUIs] When the user enters into TC...
    regex_num_C.matched -> {            // [7GUIs] ...a [non-]numerical string,
        regex_num_C.[0] =: C            // [7GUIs] ...the value in TF is [not] updated...
    }
    toString(C) => h.TC.init_text // field.content.text

    Regex regex_num_F (regex_str)
    h.TF.text =:> regex_num_F.input     // [7GUIs]...and...
    regex_num_F.matched -> {            // [7GUIs]...vice...
        regex_num_F.[0] =: F            // [7GUIs]...versa.
    }
    toString(F) => h.TF.init_text // field.content.text
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
