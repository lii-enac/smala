use core
use base
use display
use gui

import gui.widgets.PushButton
import gui.widgets.UITextField
import gui.widgets.VBox
import gui.widgets.ComboBox

// [7GUIs] Flight Booker
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#flight

// [7GUIs] Challenge: Constraints.

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
    Frame f ("mdpc", 0, 0, 600, 600)
    Exit ex (0, 1)
    f.close ->! mainloop

    // [7GUIs] The task is to build a frame containing a combobox C with the two options “one-way flight” and “return flight”,
    // [7GUIs] two textfields T1 and T2 representing the start and return date, respectively,
    // [7GUIs] and a button B for submitting the selected flight.
    
    Component model {
        List items {
          String one_way ("one-way flight")
          String return_flight ("return flight")
        }
    }
    ComboBox C
    model =: C.model
    C.preferred_width = 100
    // FIXME why is the combobox not inited on value 1 ?
    // FIXME the text fields shrink when selecting an entry in the combo box !!!!???

    UITextField T1
    UITextField T2

    PushButton B("Book")

    // [7GUIs] When clicking B a message is displayed informing the user of his selection
    // [7GUIs] (e.g. “You have booked a one-way flight on 04.04.2014.”).
    
    // [7GUIs] Initially, C has the value “one-way flight”
    // [7GUIs] and T1 as well as T2 have the same (arbitrary) date
    // [7GUIs] (it is implied that T2 is disabled).

    // FIXME does not work
    "01/01/2025" =:> T1.field.content.text //= "01/01/2025"
    T2.field.content.text = "01/01/2025"

    // [7GUIs] T2 is enabled iff C’s value is “return flight”.
    Bool one_way(0)
    C.value == "one-way flight" =:> one_way
    // [7GUIs] FIXME could be model.items[1] or .one_way -> T2 .return_flight ->! T2
    one_way.true -> T2.disable
    one_way.false -> T2.enable

    // [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red and B is disabled.
    // [7GUIs] FIXME the following validates the date even if the field is disabled

    Bool T1_valid(0)
    Bool T2_valid(0)
    Bool Book_valid(0)

    T1.field.content.text -> (root) {
        validate_date(root.T1.field.content.text, root.T1_valid)
    }
    T2.field.content.text -> (root) {
        validate_date(root.T2.field.content.text, root.T2_valid)
    }

    T1_valid.false -> {
        #FF0000 =: T1.text_color
    }
    T1_valid.true -> {
        #000000 =: T1.text_color
    }

    T1.field.content.text -> vseq: (root) {
        validate_seq(root.T1.field.content.text, root.T2.field.content.text, root.Book_valid)
    }
    T2.field.content.text -> vseq

    // [7GUIs] When C has the value “return flight” and T2’s date is strictly before T1’s then B is disabled.
    // FIXME the following does not work if C is "one-way":
    Book_valid.true -> B.enable
    Book_valid.false -> B.disable

    VBox h(f) {
        addChildrenTo h.items {
            //C, // FIXME menu not on top!!
            T1,
            T2,
            B,
            C
        }
        // FIXME now that all widgets have been reparented by addChildrenTo, they are inaccessible :-/
    }

    

}
