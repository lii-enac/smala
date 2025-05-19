use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton

// [7GUIs] Counter
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#counter

// [7GUIs] Challenge: Understanding the basic ideas of a language/toolkit.

// [7GUIs] The task is to build a frame containing a label or read-only textfield T and a button B.
// [7GUIs] Initially, the value in T is “0” and each click of B increases the value in T by one.

// [7GUIs] Counter serves as a gentle introduction to the basics of the language, paradigm and toolkit
// [7GUIs] for one of the simplest GUI applications imaginable.
// [7GUIs] Thus, Counter reveals the required scaffolding and how the very basic features work together to build a GUI application.
// [7GUIs] A good solution will have almost no scaffolding.


_main_
Component root {
    Frame f ("mdpc", 0, 0, 600, 600)
    Exit ex (0, 1)
    f.close -> ex

    // [7GUIs] The task is to build a frame containing a label or read-only textfield T and a button B.

    TextField tf (0,0, 100, 20)
    tf.text_color = #FFFFFF
    
    OutlineColor _(255,255,255)
    StandAlonePushButton count_btn("Count", 100, 0)


    // [7GUIs] Initially, the value in T is “0” and each click of B increases the value in T by one.

    // solution #1
    Int counter(0)
    count_btn.click -> {
        counter +1 =: counter
    }
    counter =:> tf.content.text

    // // solution #2
    // Incr counter(0)
    // count_btn.click -> counter
    // counter.state =:> tf.content.text
}
