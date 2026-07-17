/*
*  Smala cookbook 3-flight-booker
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2025-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/

// [7GUIs] Flight Booker
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#flight
// [7GUIs] Challenge: Constraints.


use core
use base
use display
use gui

import gui.widgets.PushButton
import gui.widgets.UITextField
import gui.widgets.VBox
import gui.widgets.ComboBox
import gui.widgets.StandAloneSelectableLabel

import Date


_main_
Component root {
    Frame f ("7GUIs Flight Booker", 800, 500, 150, 120) // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop

    ZOrderedGroup zog {
        VBox v {    
            ComboBox C              // [7GUIs] ...a combobox C...
            UITextField T1          // [7GUIs] ...two textfields T1...
            UITextField T2          // [7GUIs] ...and T2 representing the start and return date, respectively,...
            PushButton B("Book")    // [7GUIs] ...and a button B for submitting the selected flight.
        }
    }
    // make widget naming independent from layout hierarchy
    box aka zog.v
    C  aka box.C
    T1 aka box.T1
    T2 aka box.T2
    B  aka box.B

    Component cb_model {            // [7GUIs] ...with the two options “one-way flight” and “return flight”...
        List items {
          String one_way ("one-way flight")
          String return_flight ("return flight")
        }
    }
    cb_model =: C.model
    // C.preferred_width = 120
    
    cb_model.items.[1] =: C.value   // [7GUIs] Initially, C has the value “one-way flight”...

    "01.01.26" =: T1.init_text      // [7GUIs] ...and T1 as well as T2 have the same (arbitrary) date...
    "01.01.26" =: T2.init_text
    
    Bool is_return_flight(0)
    is_return_flight.true  -> T2.enable     // [7GUIs] T2 is enabled iff [if...]
    is_return_flight.false -> T2.disable    // [7GUIs] [... and only if] ...
    (C.value == "return flight") =:> is_return_flight // [7GUIs] C’s value is “return flight”.
                                                                         
    Bool T1_valid(0)
    Bool T2_valid(0)
    Bool  B_valid(0)

    Int time1(-1)
    Int time2(-1)

    Component _ { // here only to scope the following components
        Date date1(T1, T1_valid, time1)
    }

    Switch T2_sw(false) {
        Component false {
            T1_valid =:> B_valid
        }
        Component true {                                         // still boilerplate // [7GUIs] [When C has the value “return flight”]
            Date date2(T2, T2_valid, time2)
            T1_valid && T2_valid && (time1 <= time2) =:> B_valid // [7GUIs] and T2’s date is strictly before T1’s then B is disabled.
        }
    }

    is_return_flight =:> T2_sw.state  // [7GUIs] When C has the value “return flight”...

    B_valid.true  -> B.enable
    B_valid.false -> B.disable
    // "--\nT1_valid " + T1_valid + "\nT2_valid " + T2_valid + "\ntime1 " + time1 + "\ntime2 " + time2 + "\n(time1 < time2) " + to_string((time1 < time2)) + "\nB_valid " + B_valid =:> tp.input

    // # Bonus
    T1.next -> (root) {
        if (root.is_return_flight == 1) {
            notify root.T2.activate
        }
        else {
            notify root.B.select
        }

    }
    T2.next -> B.select

    Component popup_confirm_order (1) {
        Frame f2 ("", 650, 550, 400, 100)
        FillColor _ (White)
        StandAloneSelectableLabel t (10, 30, 100, 24, "NaN")
        t.width + 20 =:> f2.width
        t.height + 70 =:> f2.height
    }
    B.click -> pre_open_popup_confirm_order : (root) {
        if (root.is_return_flight) {
            root.popup_confirm_order.t.text = "You have booked a " + root.C.value + " on " + root.T1.text + " return on " + root.T2.text
        }
        else {                                                      
            root.popup_confirm_order.t.text = "You have booked a " + root.C.value + " on " + root.T1.text   // [7GUIs] ...a message is displayed informing the user of his selection 
        }   
    }
    pre_open_popup_confirm_order -> popup_confirm_order
    popup_confirm_order.f2.close ->! popup_confirm_order  
}

// original text
// [7GUIs] The task is to build a frame containing a combobox C with the two options “one-way flight” and “return flight”, 
// [7GUIs] two textfields T1 and T2 representing the start and return date, respectively, 
// [7GUIs] and a button B for submitting the selected flight. 
// [7GUIs] T2 is enabled iff C’s value is “return flight”. 
// [7GUIs] When C has the value “return flight” and T2’s date is strictly before T1’s then B is disabled. 
// [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red and B is disabled. 
// [7GUIs] When clicking B a message is displayed informing the user of his selection (e.g. “You have booked a one-way flight on 04.04.2014.”). 
// [7GUIs] Initially, C has the value “one-way flight” and T1 as well as T2 have the same (arbitrary) date (it is implied that T2 is disabled).

// [7GUIs] The focus of Flight Booker lies on modelling constraints between widgets on the one hand
// [7GUIs] and modelling constraints within a widget on the other hand.
// [7GUIs] Such constraints are very common in everyday interactions with GUI applications.

// [7GUIs] A good solution for Flight Booker will make the constraints clear, succinct and explicit in the source code and not hidden behind a lot of scaffolding.

// [7GUIs] Flight Booker is directly inspired by the Flight Booking Java example in Sodium with the simplification of using textfields for date input instead of specialized date picking widgets as the focus of Flight Booker is not on specialized/custom widgets.
