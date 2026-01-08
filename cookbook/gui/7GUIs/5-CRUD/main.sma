// [7GUIs] CRUD
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#CRUD
// [7GUIs] Challenges: separating the domain and presentation logic, managing mutation, building a non-trivial layout.

use core
use base
use display
use gui

import gui.widgets.PushButton
import gui.widgets.UITextField
import gui.widgets.VBox
import gui.widgets.HBox
import gui.widgets.ComboBox
import gui.widgets.Label

import gui.widgets.HBoxPolicy

_native_code_
%{
    #include "core/property/text_property.h"
    bool validate_date(Process *tf, Process *b_valid) {
        auto tf_text = dynamic_cast<djnn::TextProperty*> (tf);
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
    Frame f ("7GUIs CRUD - DOES NOT WORK YET", 0, 0, 600, 600)  // [7GUIs] The task is to build a frame containing the following elements:
    f.close ->! mainloop

    //_DEBUG_SEE_ACTIVATION_SEQUENCE = 1

    // [7GUIs] The layout is to be done like suggested in the screenshot. In particular, L must occupy all the remaining space.

    // screenshot:
    // Filter prefix: [     ]
    // --------------
    // | blabla     |      Name:[    ]
    // | blabla     |   Surname:[    ]
    // | blabla     |
    // --------------
    // [Create] [Update] [Delete]

    // widget identifiers:
    // L_prefix T_prefix
    // L L L L     L_name T_name
    // L L L L  L_surname Tsurname
    // L L L L
    // BC BU BD

    VBox b_all {
        HBox filter {
            Label L_prefix("Filter prefix:") // [7GUIs] the three labels as seen in the screenshot.
            UITextField T_prefix             // [7GUIs] a textfield Tprefix
        }
        HBox data {
            UITextField L                    // FIXME should be a ListBox
            VBox props {
                HBox name {
                    Label L_name("Name:")   // [7GUIs] a pair of textfields Tname and Tsurname,
                    UITextField T_name      // [7GUIs] [with a label],
                }
                HBox surname {
                    Label L_surname("Surname:") // [7GUIs] a pair of textfields Tname and Tsurname,
                    UITextField T_surname   // [7GUIs] [with a label]
                }
            }
        }
        HBox buttons {
            PushButton BC("Create") // [7GUIs] buttons BC,
            PushButton BU("Update") // [7GUIs] BU and
            PushButton BD("Delete") // [7GUIs] BD
        }
    }

    // TextPrinter tp
    // b_all.filter.T_prefix.text =:> tp.input

    // make widget naming independent from layout hierarchy
    // TODO

    // TODO !!
    
    // [7GUIs] L presents a view of the data in the database that consists of a list of names.
    // [7GUIs] At most one entry can be selected in L at a time.

    // [7GUIs] By entering a string into Tprefix the user can filter the names whose surname start with the entered prefix
    // [7GUIs] —this should happen immediately without having to submit the prefix with enter.

    // [7GUIs] Clicking BC will append the resulting name from concatenating the strings in Tname and Tsurname to L.

    // [7GUIs] BU and BD are enabled iff an entry in L is selected.

    // [7GUIs] In contrast to BC, BU will not append the resulting name but instead replace the selected entry with the new name.

    // [7GUIs] BD will remove the selected entry.
}



// [7GUIs] The task is to build a frame containing the following elements:
// [7GUIs] a textfield Tprefix, a pair of textfields Tname and Tsurname, a listbox L,
// [7GUIs] buttons BC, BU and BD and the three labels as seen in the screenshot.
// [7GUIs] L presents a view of the data in the database that consists of a list of names.
// [7GUIs] At most one entry can be selected in L at a time.
// [7GUIs] By entering a string into Tprefix the user can filter the names whose surname start with the entered prefix
// [7GUIs] —this should happen immediately without having to submit the prefix with enter.
// [7GUIs] Clicking BC will append the resulting name from concatenating the strings in Tname and Tsurname to L.
// [7GUIs] BU and BD are enabled iff an entry in L is selected.
// [7GUIs] In contrast to BC, BU will not append the resulting name but instead replace the selected entry with the new name.
// [7GUIs] BD will remove the selected entry.

// [7GUIs] The layout is to be done like suggested in the screenshot. In particular, L must occupy all the remaining space.

// [7GUIs] CRUD (Create, Read, Update and Delete) represents a typical graphical business application.
// [7GUIs] The primary challenge is the separation of domain and presentation logic in the source code
// [7GUIs] that is more or less forced on the implementer due to the ability to filter the view by a prefix.
// [7GUIs] Traditionally, some form of MVC pattern is used to achieve the separation of domain and presentation logic.
// [7GUIs] Also, the approach to managing the mutation of the list of names is tested.

// [7GUIs] A good solution will have a good separation between the domain and presentation logic without much overhead
// [7GUIs] (e.g. in the form of toolkit specific concepts or language/paradigm concepts),
// [7GUIs] a mutation management that is fast but not error-prone
// [7GUIs] and a natural representation of the layout (layout builders are allowed, of course, but would increase the overhead).

// [7GUIs] CRUD is directly inspired by the crud example in the blog post FRP - Three principles for GUI elements with bidirectional data flow.

