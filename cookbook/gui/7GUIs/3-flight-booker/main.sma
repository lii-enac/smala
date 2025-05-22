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

_native_code_
%{
    bool validate_date(Process *tf, Process *b_valid) {
        auto tf_text = dynamic_cast<TextProperty*> (tf);
        if (tf_text) {
            djnnstl::string s = tf_text ->get_value();
            // TODO
        }
        return true;
    }

    bool validate_seq(Process* tf_outbound, Process* tf_return, Process* b_valid) {
        return true;
    }
%}

_main_
Component root {
    Frame f ("7GUIs Flight Booker - DOES NOT WORK YET", 0, 0, 600, 600) // [7GUIs] The task is to build a frame containing
    f.close ->! mainloop

    ComboBox _C                 // [7GUIs] a combobox C
    UITextField _T1             // [7GUIs] two textfields T1 and T2 representing the start and return date, respectively,
    UITextField _T2
    PushButton _B("Book")       // [7GUIs] and a button B for submitting the selected flight.

    VBox h(f) {}
    addChildrenTo h.items {
        _C,
        _T1,
        _T2,
        _B
    }
    // FIXME now that all widgets have been reparented by addChildrenTo, they are inaccessible :-/
    C  aka h.items.[1]
    T1 aka h.items.[2]
    T2 aka h.items.[3]
    B  aka h.items.[4]
    
    Component model {           // [7GUIs] with the two options “one-way flight” and “return flight”,
        List items {
          String one_way ("one-way flight")
          String return_flight ("return flight")
        }
    }
    model =: C.model
    C.preferred_width = 100

    
    // FIXME why is the combobox not inited on value 1 ?
    model.items.[1] =: C.value                  // [7GUIs] Initially, C has the value “one-way flight”
    // FIXME the text fields shrink when selecting an entry in the combo box !!!!???

    // FIXME does not work
    "01/01/2025" =:> T1.text //= "01/01/2025"   // [7GUIs] and T1 as well as T2 have the same (arbitrary) date
    T2.text = "01/01/2025"
                                                // [7GUIs] (it is implied that T2 is disabled). (see below)

    
    Bool one_way(0)
    C.value == "return flight" =:> one_way
    // FIXME could be model.items[1] or .one_way -> T2 .return_flight ->! T2
    one_way.true  -> T2.enable                  // [7GUIs] T2 is enabled iff C’s value is “return flight”.
    one_way.false -> T2.disable                 // [7GUIs] T2 is enabled iff C’s value is “return flight”.

    
    // [7GUIs] (e.g. “You have booked a one-way flight on 04.04.2014.”).
    
    TextPrinter tp
    B.click -> {                                                     // [7GUIs] When clicking B 
        "You have booked a one-way flight on " + T1.text =: tp.input // [7GUIs] a message is displayed informing the user of his selection
        // FIXME? popup?
    }

    
    // [7GUIs] FIXME the following validates the date even if the field is disabled

    Bool T1_valid(0)
    Bool T2_valid(0)
    Bool Book_valid(0)

    T1.text -> (root) {
        validate_date(root.T1.text, root.T1_valid) // validate_date will set T1_valid
    }
    T1.text ~> T1_valid // since validate_date modifies T1_valid from T1_text, declare a causal relationship
    T2.text -> (root) {
        validate_date(root.T2.text, root.T2_valid) // idem
    }
    T2.text ~> T2_valid // idem

    T1_valid.false -> {
        #FF0000 =: T1.text_color                // [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red
    }
    T1_valid.false -> B.disable                   // [7GUIs] and B is disabled.
    T1_valid.true -> {
        #000000 =: T1.text_color
    }


    T1.text -> vseq: (root) {
        validate_seq(root.T1.text, root.T2.text, root.Book_valid)
    }
    T2.text -> vseq

    // [7GUIs] When C has the value “return flight” and T2’s date is strictly before T1’s then B is disabled.
    // FIXME the following does not work if C is "one-way":
    Book_valid.true -> B.enable
    Book_valid.false -> B.disable
}



// [7GUIs] The task is to build a frame containing a combobox C with the two options “one-way flight” and “return flight”,
// [7GUIs] two textfields T1 and T2 representing the start and return date, respectively,
// [7GUIs] and a button B for submitting the selected flight.
// [7GUIs] T2 is enabled iff C’s value is “return flight”.
// [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red and B is disabled.
// [7GUIs] When C has the value “return flight” and T2’s date is strictly before T1’s then B is disabled.

// [7GUIs] The focus of Flight Booker lies on modelling constraints between widgets on the one hand
// [7GUIs] and modelling constraints within a widget on the other hand.
// [7GUIs] Such constraints are very common in everyday interactions with GUI applications.

// [7GUIs] A good solution for Flight Booker will make the constraints clear, succinct and explicit in the source code and not hidden behind a lot of scaffolding.

// [7GUIs] Flight Booker is directly inspired by the Flight Booking Java example in Sodium with the simplification of using textfields for date input instead of specialized date picking widgets as the focus of Flight Booker is not on specialized/custom widgets.

