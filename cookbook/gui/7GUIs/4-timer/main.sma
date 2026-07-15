/*
*  djnn Smala compiler
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
// [7GUIs] Timer
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#timer
// [7GUIs] Challenges: concurrency, competing user/signal interactions, responsiveness.

use core
use base
use display
use gui

import gui.widgets.PushButton
import gui.widgets.HSlider
import gui.widgets.Label
import gui.widgets.ProgressBar
import gui.widgets.VBox
import gui.widgets.HBox

_main_
Component root {
    Frame f ("7GUIs Timer")             // [7GUIs] The task is to build a frame containing
    f.close ->! mainloop
    mouseTracking = 1    // FIXME otherwise Slider adjustment won't work

    VBox v_box {
        v_box.space = 20

        HBox h_box1 {
            Label L1 ("Elapsed time: ")
            ProgressBar G (0)           // [7GUIs] a gauge G for the elapsed time e,
        }
        // FillColor _(#FFFFFF)         // FIXME how to control color once reparented???
        Label L ("0s")                  // [7GUIs] a label [L] which shows the elapsed time as a numerical value,
        L.h_alignment = 0

        HBox h_box2 {
            Label L2 ("Duration: ")
            HSlider S (50)              // [7GUIs] a slider S by which the duration d of the timer can be adjusted while the timer is running and
        }
        h_box2.h_alignment = 0
        
        PushButton R ("Reset")          // [7GUIs] a reset button R.
    }
    // make widget naming independent from layout hierarchy
    G aka v_box.h_box1.G
    L aka v_box.L
    S aka v_box.h_box2.S
    R aka v_box.R


    Double d(0)             // [7GUIs] ...duration...
    Double e(0)             // [7GUIs] ...elapsed...

    Clock cl(100)           // [7GUIs] ...timer...  // (there is a Timer Process in djnn, but with a different semantic)
    cl.tick -> { e + 0.1 =: e }

    // bv is the bounded value of the Gauge // FIXME? isn't this too dependent on the implementation? or not...
    d * 10 =:> G.bv.max     // [7GUIs] a gauge G for the elapsed time e
    e * 10 =:> G.bv.input
    
    e + "s" =:> L.text      // [7GUIs] a label [L] which shows the elapsed time as a numerical value,

    S.value / 10 =:> d      // [7GUIs] Adjusting S must immediately reflect on d and not only when S is released.

    // see above:           // [7GUIs] It follows that while moving S the filled amount of G will (usually) change immediately.
    // d * 10 =:> G.bv.max will be triggered when d is modified

    Bool stop_cond(0)
    e >= d =:> stop_cond    // [7GUIs] When e ≥ d ...
    stop_cond.true  ->! cl  // [7GUIs] ... is true then the timer stops (and G will be full).
    stop_cond.false ->  cl  // [7GUIs] If, thereafter, d is increased such that d > e will be true then the timer restarts to tick until e ≥ d is true again.

    R.click -> { 0 =: e }   // [7GUIs] Clicking R will reset e to zero.
}


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
