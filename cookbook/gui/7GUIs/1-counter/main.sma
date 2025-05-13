use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton


_main_
Component root {
    Frame f ("mdpc", 0, 0, 600, 600)
    Exit ex (0, 1)
    f.close -> ex
    f.background_color.r = 128
    f.background_color.g = 128
    f.background_color.b = 128

    TextField tf (0,0, 100, 20)
    StandAlonePushButton count_btn("Count", 0, 100)

    // solution 1
    Int counter(0)
    count_btn.click -> {
        counter +1 =: counter
    }
    counter =:> tf.content.text

    // // solution 2
    // Incr counter(0)
    // count_btn.click -> counter
    // counter.state =:> tf.content.text
}
