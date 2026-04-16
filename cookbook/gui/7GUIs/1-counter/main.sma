// [7GUIs] Counter
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#counter
// [7GUIs] Challenge: Understanding the basic ideas of a language/toolkit.

use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton


_main_
Component root {
    Frame f("7GUIs Counter")                    // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop

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




// [7GUIs] The task is to build a frame containing a label or read-only textfield T and a button B.
// [7GUIs] Initially, the value in T is “0” and each click of B increases the value in T by one.

// [7GUIs] Counter serves as a gentle introduction to the basics of the language, paradigm and toolkit
// [7GUIs] for one of the simplest GUI applications imaginable.
// [7GUIs] Thus, Counter reveals the required scaffolding and how the very basic features work together to build a GUI application.

// [7GUIs] A good solution will have almost no scaffolding.
