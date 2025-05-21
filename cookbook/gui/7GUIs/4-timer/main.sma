// [7GUIs] Timer
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#timer

// [7GUIs] The task is to build a frame containing a gauge G for the elapsed time e,
// [7GUIs] a label which shows the elapsed time as a numerical value,
// [7GUIs] a slider S by which the duration d of the timer can be adjusted while the timer is running and
// [7GUIs] a reset button R.

// [7GUIs] Adjusting S must immediately reflect on d and not only when S is released.
// [7GUIs] It follows that while moving S the filled amount of G will (usually) change immediately.
// [7GUIs] When e ≥ d is true then the timer stops (and G will be full).
// [7GUIs] If, thereafter, d is increased such that d > e will be true then the timer restarts to tick until e ≥ d is true again.
// [7GUIs] Clicking R will reset e to zero.

// [7GUIs] Timer deals with concurrency in the sense that a timer process that updates the elapsed time
// [7GUIs] runs concurrently to the user’s interactions with the GUI application.
// [7GUIs] This also means that the solution to competing user and signal interactions is tested.
// [7GUIs] The fact that slider adjustments must be reflected immediately moreover tests the responsiveness of the solution.

// [7GUIs] A good solution will make it clear that the signal is a timer tick and, as always, has not much scaffolding.

// [7GUIs] Timer is directly inspired by the timer example in the paper
// [7GUIs] Crossing State Lines: Adapting Object-Oriented Frameworks to Functional Reactive Languages.

use core
use base
use display
use gui

import gui.widgets.PushButton
import gui.widgets.HSlider
import gui.widgets.Label
import gui.widgets.VBox
import gui.widgets.HBox

_main_
Component root {
    // [7GUIs] The task is to build a frame containing    
    Frame f ("7GUIs Timer", 0, 0, 600, 600)
    f.close ->! mainloop
    mouseTracking = 1 // FIXME otherwise Slider adjustment won't work

    // [7GUIs] a gauge G for the elapsed time e,
    HSlider _G(50) // FIXME should be a Gauge

    // [7GUIs] a label [L] which shows the elapsed time as a numerical value,
    FillColor _(#FFFFFF) // FIXME how to control color once reparented???
    Label _L("empty")

    // [7GUIs] a slider S by which the duration d of the timer can be adjusted while the timer is running and
    HSlider _S(50) // FIXME adjusting does not work?!!!!!!!!

    // [7GUIs] a reset button R.
    PushButton _R("Reset")

    VBox box(f) {}
    addChildrenTo box.items {
        _G,
        _L,
        _S,
        _R
    }
    // FIXME now that all widgets have been reparented by addChildrenTo, they are inaccessible :-/
    G aka box.items.[1]
    L aka box.items.[2]
    S aka box.items.[3]
    R aka box.items.[4]


    Int d(0) // duration
    Int e(0) // elapsed

    Clock cl(1000) // timer (there is a Timer Process in djnn, but with a different semantic)
    cl.tick -> { e + 1 =: e }

    // [7GUIs] a gauge G for the elapsed time e
    d * 10 =:> G.bv.max
    e * 10 =:> G.bv.input
    // here bv is the bounded value of the Gauge
    
    // [7GUIs] a label [L] which shows the elapsed time as a numerical value,
    e =:> L.text
    
    // [7GUIs] Adjusting S must immediately reflect on d and not only when S is released.
    S.value / 10 =:> d

    // [7GUIs] It follows that while moving S the filled amount of G will (usually) change immediately.
    // see above: d * 10 =:> G.bv.max, which will be triggered when d is modified

    // [7GUIs] When e ≥ d is true then the timer stops (and G will be full).
    Bool stop_cond(0)
    e >= d =:> stop_cond
    stop_cond.true  ->! cl
    // [7GUIs] If, thereafter, d is increased such that d > e will be true then the timer restarts to tick until e ≥ d is true again.
    stop_cond.false ->  cl 

    // [7GUIs] Clicking R will reset e to zero.
    R.click -> { 0 =: e }

}
