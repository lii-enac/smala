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
    Frame f ("mdpc", 0, 0, 600, 600)
    Exit ex (0, 1)
    f.close -> ex
    f.background_color.r = 128
    f.background_color.g = 128
    f.background_color.b = 128

    Double C(0)
    Double F(0)

//  (F - 32) * (5/9) => C // FIXME CYCLE
      C * (9/5) + 32 => F

    UITextField celsius
    Label c_text(" celsius =")
    Label f_text(" farenheit")
    UITextField farenheit

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
