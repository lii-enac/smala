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

// import gui.widgets.HBoxPolicy

import gui.widgets.ListViewer

import PersonModel
import PersonView

_native_code_
%{
    #include "core/property/text_property.h"

%}

_main_
Component root {
    Frame f ("7GUIs CRUD - DOES NOT WORK YET", 0, 0, 600, 600)  // [7GUIs] The task is to build a frame containing the following elements:
    f.close ->! mainloop

    //_DEBUG_SEE_ACTIVATION_SEQUENCE = 1
    // _DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 1

    TextPrinter tp

    List models
    models.size + " persons in the list" => tp.input

    // FIXME: v0.1
    // Component prototype (1) {
    //     PersonView view
    // }

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
        b_all.space = 20

        HBox filter {
            Label L_prefix("Filter prefix:") // [7GUIs] the three labels as seen in the screenshot.
            UITextField T_prefix             // [7GUIs] a textfield Tprefix
        }
        filter.h_alignment = 0

        HBox data {
            data.space = 20
            
            // FIXME: v0.1
            // ListViewer list_viewer (models, prototype.view)

            VBox L {}                       // [7GUIs] L presents a view of the data in the database that consists of a list of names.

            VBox props {
                HBox name {
                    Label L_name("Name:")   // [7GUIs] a pair of textfields Tname and Tsurname,
                    L_name.preferred_width = 60
                    UITextField T_name      // [7GUIs] [with a label],
                }
                HBox surname {
                    Label L_surname("Surname:") // [7GUIs] a pair of textfields Tname and Tsurname,
                    L_surname.preferred_width = 60
                    UITextField T_surname   // [7GUIs] [with a label]
                }
            }
        }
        HBox buttons {
            PushButton BC("Create") // [7GUIs] buttons BC,
            PushButton BU("Update") // [7GUIs] BU and
            PushButton BD("Delete") // [7GUIs] BD
        }
        buttons.h_alignment = 0
    }

    // b_all.filter.T_prefix.text =:> tp.input

    // make widget naming independent from layout hierarchy
    L aka b_all.data.L
    T_name aka b_all.data.props.name.T_name
    T_surname aka b_all.data.props.surname.T_surname
    BC aka b_all.buttons.BC
    BU aka b_all.buttons.BU
    BD aka b_all.buttons.BD


    // By default, all buttons are disabled
    |-> BC.disable, BU.disable, BD.disable

    // TODO !!

    // FIXME: for test
    // PersonModel temp_person ("Vince", "Pey")
    
    // [7GUIs] L presents a view of the data in the database that consists of a list of names.
    // [7GUIs] At most one entry can be selected in L at a time.

    // [7GUIs] By entering a string into Tprefix the user can filter the names whose surname start with the entered prefix
    // [7GUIs] —this should happen immediately without having to submit the prefix with enter.

    // [7GUIs] Clicking BC will append the resulting name from concatenating the strings in Tname and Tsurname to L.
    Bool is_missing_value (true)
    (getString (T_name.text) == "") || (getString (T_surname.text) == "") => is_missing_value

    BC.click -> na_create_person:(root) {
        // print ("Click on BC with " + root.T_name.text + " " + root.T_surname.text)
        addChildrenTo root.models {
            PersonModel _ (getString (root.T_name.text), getString (root.T_surname.text))
        }
    }
    na_create_person -> root.T_name.clear, root.T_surname.clear
    
    BD.click -> na_delete_person:(root) {
        if (root.models.size > 0) {
            Process model = &root.models.[root.models.size]

            // We cannot delete the model yet
            //delete model
            // Only remove from list
            remove model from root.models
        }
    }

    is_missing_value.false -> BC.enable
    is_missing_value.true -> BC.disable

    models.size > 0 -> BD.enable
    models.size == 0 -> BD.disable

    // Key "tab" allows to set the focus to next text field
    T_name.next -> T_surname.activate
    T_surname.next -> BC.select

    // T_name.text + " & " + T_surname.text + " --> is_missing_value ?= " + is_missing_value =:> tp.input

    // [7GUIs] BU and BD are enabled iff an entry in L is selected.

    // [7GUIs] In contrast to BC, BU will not append the resulting name but instead replace the selected entry with the new name.

    // [7GUIs] BD will remove the selected entry.

    models.$added -> na_added:(root) {
        model = getRef (&root.models.$added)
        if (&model != null)
        {
            print ("The model '" + model.fullname + "' has been added to the list")
            addChildrenTo root.L {
                Label _ (getString (model.fullname))
                // How to link this model and this view to update it when model change ?
                // model.fullname =:> _.text
            }

            notify root.L.pack
        }
    }
    
    models.$removed -> na_removed:(root) {
        model = getRef (&root.models.$removed)
        if (&model != null)
        {
            print ("The model '" + model.fullname + "' has been removed from the list")

            for view : root.L.items {
                // print ("view of " + view.model.fullname)
                print ("view of " + view.text)
            }
            // delete model
        }        
    }
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

