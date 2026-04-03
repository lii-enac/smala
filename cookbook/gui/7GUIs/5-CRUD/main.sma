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
import gui.keyboard.ControlKey

import ListBox
import ListBoxItem

import PersonModel
import PersonView

_native_code_
%{
    #include "core/property/text_property.h"

    using namespace djnnstl;

    bool starts_with (const string &str, const string &prefix) {
      return str.starts_with (prefix);  // C++20
    }

    bool contains (Process* collection, Process* item) {
        ProcessCollector* pc = dynamic_cast<ProcessCollector*> (collection);
        if ( (pc != nullptr) && (item != nullptr) ) {
            for (CoreProcess* item_i : pc->get_list ()) {
                if (item_i == item) {
                    return true;
                }
            }
        }
        return false;
    }

    void remove_one (Process* collection, Process* item) {
        ProcessCollector* pc = dynamic_cast<ProcessCollector*> (collection);
        if ( (pc != nullptr) && (item != nullptr) ) {
            pc->remove_one (item);
        }
    }

    void my_print (const string& s1, const string& s2) {
        cout << s1 << s2 << endl;
    }

%}


_action_
action_model_added_to_all_models (Process src, Process self)
{
    model = getRef (&self.models.$added)
    if (&model != null)
    {
        print ("The model '" + model.fullname + "' has been added to the list of all " + self.models.size + " models")

        // By default, add to the list of displayed_models
        
        // FIXME TODO: check if T_prefix.field.content.text != ""
        // string filter = getString (self.T_prefix.field.content.text)
        
        setRef (self.displayed_models.add, model)
    }
}


_action_
action_model_added_to_displayed_models (Process src, Process self)
{
    model = getRef (&self.displayed_models.add)
    if (&model != null)
    {
        print ("The model '" + model.fullname + "' has been added to the list of " + self.displayed_models.size + " models to display")

        // Find the index of this model in the full list "models" & use it to insert at the right place in the views list
        int index = 0
        for model_i : self.models {
            if (&model_i == &model) {   // found
                break
            }
            index++
        }
        if (index == self.models.size - 1) {
            my_print ("Add at the end: ", to_string (index))
        }
        else {
            my_print ("Insert at: ", to_string (index))
        }

        addChildrenTo self.L {
            //ListBoxItem item (model, index)
            PersonView item (model, index)
        }
        notify self.L.pack  // Update layout
    }
}


_action_
action_model_removed_from_displayed_models (Process src, Process self)
{
    model = getRef (&self.displayed_models.rm)
    if (&model != null)
    {
        print ("The model '" + model.fullname + "' has been removed from the list of " + self.displayed_models.size + " models to display")

        for view : self.L.items {
            if (&view.model == &model) {
                print ("We found the view of the removed model " + view.model.fullname)
                
                if (view.is_selected) {
                    notify self.L.reset_selection
                }

                // Remove from collection
                remove_one (self.L.items, view)
                // Remove from children list
                remove view from self.L

                // Delete view
                delete view

                break
            }
        }
        // notify self.L.pack  // Update layout
    }
}


_action_
action_filter (Process src, Process self)
{
    print ("Filter list with prefix '" + src + "'")
    string prefix = getString (src)
    
    for model : self.models {
        // Surname STARTS with prefix
        if (starts_with (getString (model.surname), prefix)) {
            
            if (!contains (self.displayed_models, model)) {
                print ("+ " + model.surname + " STARTS with prefix (" + src +") & NOT in displayed list --> add it")

                // Add to the list of displayed_models
                setRef (self.displayed_models.add, model)
                graph_exec ()
            }
        }
        // Surname does NOT start with prefix
        else {
            if (contains (self.displayed_models, model)) {
                print ("- " + model.surname + " does NOT start with prefix (" + src + ") & already IN displayed list --> remove it")

                // Remove from the list of displayed_models
                setRef (self.displayed_models.rm, model)
                graph_exec ()
            }
        }
    }
}


_action_
action_delete_selected_person (Process src, Process self)
{
    selected_item = getRef (self.L.selected_item)
    if (&selected_item != null) {
        print ("Delete view & model of " + selected_item.text)

        notify self.L.reset_selection                   // Reset the selection
        
        Process model = find (selected_item, "model")   // Get the corresponding model

        remove_one (self.L.items, selected_item)        // Remove view from collection
        remove selected_item from self.L                // Remove view from children list

        if (&model != null) {
            if (contains (self.displayed_models, model)) {
                // setRef (root.displayed_models.rm, model)
                // graph_exec ()
                remove_one (self.displayed_models, model)   // We have to remove from the list of displayed_models immediately !
            }

            delete selected_item            // Delete view

            // notify self.L.pack           // Update layout

            remove model from self.models   // Remove from list of all models
            delete model                    // Delete the model
        }
    }
}


_main_
Component root {
    Frame f ("7GUIs CRUD - DOES NOT WORK YET", 0, 0, 600, 600)  // [7GUIs] The task is to build a frame containing the following elements:
    f.close ->! mainloop

    //_DEBUG_SEE_ACTIVATION_SEQUENCE = 1
    _DEBUG_SEE_PROP_SET_VALUE = 0
    _DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 0

    ControlKey ctrl_d (f, DJN_Key_D)

	ctrl_d.press -> (root) {
		if (_DEBUG_SEE_ACTIVATION_SEQUENCE_2) {
			_DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 0
			_DEBUG_SEE_PROP_SET_VALUE = 0
		}
		else {
			_DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 1
			_DEBUG_SEE_PROP_SET_VALUE = 1
		}
	}

    TextPrinter tp
    // TextPrinter tp_forename
    // TextPrinter tp_surename
    TextPrinter tp_selection

    List models // List of all models
    ProcessCollector displayed_models
    displayed_models.size + " displayed models among the " + models.size + " persons in the full list" => tp.input


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

            // VBox L {}                       // [7GUIs] L presents a view of the data in the database that consists of a list of names.
            // ListBox L (models) { }
            ListBox L () { }
            // L.preferred_width = 200
            // 200 =: L.preferred_width
            // L.min_width = 250
            // 250 =: L.min_width
            // L.min_height = 450

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

    // make widget naming independent from layout hierarchy
    T_prefix aka b_all.filter.T_prefix
    L aka b_all.data.L
    T_name aka b_all.data.props.name.T_name
    T_surname aka b_all.data.props.surname.T_surname
    BC aka b_all.buttons.BC
    BU aka b_all.buttons.BU
    BD aka b_all.buttons.BD

    DerefString name_of_selected_item (L.selected_item, "model/name", DJNN_GET_ON_CHANGE)
    DerefString surname_of_selected_item (L.selected_item, "model/surname", DJNN_GET_ON_CHANGE)

    // By default, all buttons are disabled
    |-> BC.disable, BU.disable, BD.disable

    
    // [7GUIs] L presents a view of the data in the database that consists of a list of names.
    // [7GUIs] At most one entry can be selected in L at a time.

    NativeAction na_model_added_to_all_models (action_model_added_to_all_models, root, 1)
    models.$added -> na_model_added_to_all_models

    // models.$added -> na_added:(root) {
    //     model = getRef (&root.models.$added)
    //     if (&model != null)
    //     {
    //         print ("The model '" + model.fullname + "' has been added to the list of all " + root.models.size + " models")

    //         // By default, add to the list of displayed_models
    //         // FIXME TODO: check if T_prefix.field.content.text != ""
    //         setRef (root.displayed_models.add, model)
    //     }
    // }

    NativeAction na_model_added_to_displayed_models (action_model_added_to_displayed_models, root, 1)
    displayed_models.add -> na_model_added_to_displayed_models

    // displayed_models.add -> na_model_to_display:(root) {
    //     model = getRef (&root.displayed_models.add)
    //     if (&model != null)
    //     {
    //         print ("The model '" + model.fullname + "' has been added to the list of " + root.displayed_models.size + " models to display")

    //         // Find the index of this model in the full list "models" & use it to insert at the right place in the views list
    //         int index = 0
    //         for model_i : root.models {
    //             if (&model_i == &model) {   // found
    //                 break
    //             }
    //             index++
    //         }
    //         if (index == root.models.size - 1) {
    //             my_print ("Add at the end: ", to_string (index))
    //         }
    //         else {
    //             my_print ("Insert at: ", to_string (index))
    //         }

    //         addChildrenTo root.L {
    //             //ListBoxItem item (model, index)
    //             PersonView item (model, index)
    //         }
    //         notify root.L.pack  // Update layout
    //     }
    // }

    NativeAction na_model_removed_from_displayed_models (action_model_removed_from_displayed_models, root, 1)
    displayed_models.rm -> na_model_removed_from_displayed_models

    // displayed_models.rm -> na_model_to_hide:(root) {
    //     model = getRef (&root.displayed_models.rm)
    //     if (&model != null)
    //     {
    //         print ("The model '" + model.fullname + "' has been removed from the list of " + root.displayed_models.size + " models to display")

    //         for view : root.L.items {
    //             if (&view.model == &model) {
    //                 print ("We found the view of the removed model " + view.model.fullname)
                   
    //                 if (view.is_selected) {
    //                     notify root.L.reset_selection
    //                 }

    //                 // Remove from collection
    //                 remove_one (root.L.items, view)
    //                 // Remove from children list
    //                 remove view from root.L

    //                 // Delete view
    //                 delete view

    //                 break
    //             }
    //         }
    //         // notify root.L.pack  // Update layout
    //     }
    // }


    // L.selected_item -> {
    (L.selected_item.is_null == 0) && L.selected_item -> {
        name_of_selected_item.value =: root.T_name.init_text
        surname_of_selected_item.value =: root.T_surname.init_text
    }
    // NO selection -> clear text fields
    L.selected_item.is_null.true -> root.T_name.clear, root.T_surname.clear

    // [7GUIs] By entering a string into Tprefix the user can filter the names whose surname start with the entered prefix
    // [7GUIs] —this should happen immediately without having to submit the prefix with enter.

    // TextPrinter tp_prefix
    // // "Filter list with prefix " + T_prefix.text => tp_prefix.input
    // "Filter list with prefix " + T_prefix.field.content.text => tp_prefix.input

    NativeAction na_filter (action_filter, root, 1)
    T_prefix.field.content.text -> na_filter

    // T_prefix.field.content.text -> na_filter:(root) {
    //     print ("Filter list with prefix '" + root.T_prefix.field.content.text + "'")
    //     string prefix = getString (root.T_prefix.field.content.text)
        
    //     for model : root.models {
    //         // Surname STARTS with prefix
    //         if (starts_with (getString (model.surname), prefix)) {
                
    //             if (!contains (root.displayed_models, model)) {
    //                 print ("+ " + model.surname + " STARTS with prefix & NOT in displayed list --> add it")

    //                 // Add to the list of displayed_models
    //                 setRef (root.displayed_models.add, model)
    //                 graph_exec ()
    //             }
    //         }
    //         // Surname does NOT start with prefix
    //         else {
    //             if (contains (root.displayed_models, model)) {
    //                 print ("- " + model.surname + " does NOT start with prefix & IN displayed list --> remove it")

    //                 // Remove from the list of displayed_models
    //                 setRef (root.displayed_models.rm, model)
    //                 graph_exec ()
    //             }
    //         }
    //     }
    // }


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
    
    is_missing_value.false -> BC.enable
    is_missing_value.true -> BC.disable

    // Key "tab" allows to set the focus to next text field
    T_name.next -> T_surname.activate
    T_surname.next -> BC.select

    // T_name.text + " & " + T_surname.text + " --> is_missing_value ?= " + is_missing_value =:> tp.input

    // [7GUIs] BU and BD are enabled if an entry in L is selected.
    L.selected_item.is_null.false -> BU.enable, BD.enable
    L.selected_item.is_null.true -> BU.disable, BD.disable


    // [7GUIs] In contrast to BC, BU will not append the resulting name but instead replace the selected entry with the new name.
    // (is_missing_value == 0) && BU.click -> {
    BU.click -> {
        // "Update (fore)name: " + name_of_selected_item.value + " -> " + root.T_name.text =: tp_forename.input
        // "Update surname: " + surname_of_selected_item.value + " -> " + root.T_surname.text =: tp_surename.input
        root.T_name.text =?: name_of_selected_item.value
        root.T_surname.text =?: surname_of_selected_item.value
    }

    // [7GUIs] BD will remove the selected entry.

    NativeAction na_delete_selected_person (action_delete_selected_person, root, 1)
    BD.click -> na_delete_selected_person

    // BD.click -> na_delete_person:(root) {
    //     selected_item = getRef (root.L.selected_item)
    //     if (&selected_item != null) {
    //         print ("Delete view of " + selected_item.text)

    //         notify root.L.reset_selection
            
    //         Process model = find (selected_item, "model")

    //         // Remove from collection
    //         remove_one (root.L.items, selected_item)
    //         // Remove from children list
    //         remove selected_item from root.L

    //         if (&model != null) {
    //             print ("Delete its model " + model.fullname)
                
    //             if (contains (root.displayed_models, model)) {
    //                 // We have to remove from the list of displayed_models immediately !
    //                 // setRef (root.displayed_models.rm, model)
    //                 // graph_exec ()
    //                 remove_one (root.displayed_models, model)
    //             }

    //             // Only remove from list of all models
    //             remove model from root.models

    //             // Delete view
    //             delete selected_item

    //             // notify root.L.pack  // Update layout

    //             // Delete the model
    //             delete model
    //         }
    //     }
    // }



    // FIXME: for tests / debug
    Int DEBUG_Index (0)
    f.key\-pressed -> na_key_pressed:(root) {
        // Key +
        if (root.f.key\-pressed == "43") {
            root.DEBUG_Index++
            // print ("+ --> " + root.DEBUG_Index)
            string nb = to_string (getInt (root.DEBUG_Index))

            addChildrenTo root.models {
                PersonModel _ ("prenom_" + nb, "nom_" + nb)
            }
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

