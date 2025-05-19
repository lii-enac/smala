use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton
import gui.widgets.UITextField
import gui.widgets.HBox
import gui.widgets.Label

// [7GUIs] Temperature Converter
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#temp

// [7GUIs] Challenges: bidirectional data flow, user-provided text input.

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
// ([SCO] also a famous example from Fabrik)
// [7GUIs] It is such a widespread example—sometimes also in the form of a currency converter—that one could give a thousand references.
// ([SCO] inc. those of Jef Raskin)
// [7GUIs] The same is true for the Counter task.



_main_
Component root {
    Frame f ("mdpc", 0, 0, 600, 600)
    Exit ex (0, 1)
    f.close -> ex

    Double C(0)
    Double F(0)

    // [7GUIs] The formula for converting a temperature C in Celsius into a temperature F in Fahrenheit is C = (F - 32) * (5/9) and the dual direction is F = C * (9/5) + 32.
//  (F - 32) * (5/9) => C // FIXME CYCLE
      C * (9/5) + 32 => F


    // [7GUIs] The task is to build a frame containing two textfields TC and TF representing the temperature in Celsius and Fahrenheit, respectively.
    // [7GUIs] Initially, both TC and TF are empty.

    UITextField celsius
    Label c_text(" celsius =")
    Label f_text(" farenheit")
    UITextField farenheit


    // [7GUIs] When the user enters a numerical value into TC the corresponding value in TF is automatically updated and vice versa.
    // [7GUIs] When the user enters a non-numerical string into TC the value in TF is not updated and vice versa.

    celsius.text =:> C
    toString(C) => celsius.field.content.text

    farenheit.text =:> F
    toString(F) => farenheit.field.content.text

    TextPrinter tp
    "" + C + " " + F + " " + farenheit.text =:> tp.input

    HBox h(f) {
        addChildrenTo h.items {
            celsius,
            c_text,
            farenheit,
            f_text
        }
    }
}
