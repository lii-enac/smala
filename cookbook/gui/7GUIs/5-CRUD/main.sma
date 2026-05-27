// [7GUIs] CRUD
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#CRUD
// [7GUIs] Challenges: separating the domain and presentation logic, managing mutation, building a non-trivial layout.

use core
use base
use display
use gui

// import gui.keyboard.ControlKey
import gui.widgets.PushButton
import gui.widgets.UITextField
import gui.widgets.VBox
import gui.widgets.HBox
import gui.widgets.Label
import gui.widgets.ListBox
// import gui.widgets.ListBoxItem

import PersonModel
import PersonView

_native_code_
%{
    #include "core/property/text_property.h"

    using namespace djnnstl;

    bool starts_with (const string &str, const string &prefix) {
      return str.starts_with (prefix);  // C++20
    }

    // Allow to call ProcessCollector::contains (CoreProcess* p) directly from smala
    bool contains (Process* collection, Process* item) {
        ProcessCollector* pc = dynamic_cast<ProcessCollector*> (collection);
        if ( (pc != nullptr) && (item != nullptr) ) {
            return pc->contains (item);
        }
        return false;
    }

    // Allow to call ProcessCollector::remove_one (CoreProcess* p) directly from smala
    void remove_one (Process* collection, Process* item) {
        ProcessCollector* pc = dynamic_cast<ProcessCollector*> (collection);
        if ( (pc != nullptr) && (item != nullptr) ) {
            pc->remove_one (item);
        }
    }

    // Get the index of a model in a vector
    size_t get_index (Container* lst_models, Process* model)
    {
        if ( (lst_models != nullptr) && (model != nullptr) )
        {
            const vector<CoreProcess*>& models = lst_models->children ();
            auto it = std::find (models.begin(), models.end(), model);
            if (it == models.end())
                return -1;
            return std::distance(models.begin(), it);
        }
        return -1;
    }

    // Create a view corresponding to the model to display
    // & insert it in the list box at the right index (to correspond to the ordered list with all models)
    void insert_view_for_model_to_display (Process* model, Process* ordered_models, Process* list_box)
    {
        const string& surname = static_cast<TextProperty*>(model->find_child("surname"))->get_value();
        // cout << "\nInsert view of " << surname << " in displayed views" << endl;
        Container* lst_models = dynamic_cast<Container*> (ordered_models);

        // Get the index of this model in the list of all models (to use it as reference for comparison)
        size_t ref_index = get_index (lst_models, model);
        if (ref_index == -1)
            return; 

        ProcessCollector* pc_items = dynamic_cast<ProcessCollector*> (list_box->find_child ("items"));
        if (pc_items != nullptr)
        {
            const vector<CoreProcess*>& displayed_items = pc_items->get_list ();

            auto insert_pos = displayed_items.end();    // At the end by default

            // Browse list_box.items to find where to insert
            for (auto it = displayed_items.begin(); it != displayed_items.end(); ++it)
            {
                Process* model_i = (*it)->find_child ("model");
                size_t current_index = get_index (lst_models, model_i); // Get the index of the current model in the list of all models
                if (current_index > ref_index) {                        // If this current index > reference index...
                    insert_pos = it;                                    // Insert before this item
                    break;                                              // Exit loop
                }
            }
            size_t insert_index = std::distance(displayed_items.begin(), insert_pos);

            // Create the view without parent !
            // 4: Horizontal margin on the left & on the right of the label
            // 18: height of the list box item
            Process* view = PersonView (nullptr, surname, model, 4, 18);

            // Insert the view in the ProcessCollector "list_box.items"
            // pc_items->insert_one (view, insert_pos);
            pc_items->insert_one (view, insert_index);

            // Finally, add the view as child of the list box (at the end but it doesn't matter)
            list_box->add_child (view, surname);
        }
    }
%}


_action_
action_model_removed_from_displayed_models (Process src, Process self)
{
    model = getRef (&self.displayed_models.rm)
    if (&model != null)
    {
        // print ("The model '" + model.surname + "' has been removed from the list of " + self.displayed_models.size + " models to display")

        for view : self.L.items {
            if (&view.model == &model) {            // We found the view of the removed model
                if (view.is_selected) {
                    notify self.L.reset_selection       // Reset the selection // drawback: not visible in the main code
                    // activate (self.L.reset_selection)   // Replace notify by activate (...) ?
                }

                remove_one (self.L.items, view)     // Remove from the ProcessCollector "list_box.items"
                remove view from self.L             // Remove from children list of the list box

                delete view                         // Then, delete the view

                break                               // exit loop for
            }
        }
    }
}


_action_
action_filter (Process src, Process self)
{
    // print ("Filter list with prefix '" + src + "'")
    string prefix = getString (src)
    
    for model : self.models {
        // Surname STARTS with prefix
        if (starts_with (getString (model.surname), prefix)) {
            
            // The surname STARTS with the prefix & NOT in displayed list --> add it")
            if (!contains (self.displayed_models, model)) {
                // Add to the list of displayed_models
                setRef (self.displayed_models.add, model)
                graph_exec ()
            }
        }
        // Surname does NOT start with prefix
        else {
            // The surname does NOT start with the prefix & already IN displayed list --> remove it")
            if (contains (self.displayed_models, model)) {
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
        // print ("Delete view & model of " + selected_item.text)

        notify self.L.reset_selection                   // Reset the selection // drawback: not visible in the main code
        // activate (self.L.reset_selection)               // Replace notify by activate (...) ?
        
        Process model = find (selected_item, "model")   // Get the corresponding model

        remove_one (self.L.items, selected_item)        // Remove view from collection "items"
        remove selected_item from self.L                // Remove view from children list

        if (&model != null) {
            if (contains (self.displayed_models, model)) {  // Could be filtered out
                // setRef (root.displayed_models.rm, model)
                // graph_exec ()
                remove_one (self.displayed_models, model)   // We have to remove from the list of displayed_models immediately !
            }

            delete selected_item            // Delete view

            remove model from self.models   // Remove from list of all models
            delete model                    // Delete the model
        }
    }
}


_main_
Component root {
    Frame f ("7GUIs CRUD", 0, 0, 600, 600)  // [7GUIs] The task is to build a frame containing the following elements:
    f.close ->! mainloop

    // //_DEBUG_SEE_ACTIVATION_SEQUENCE = 1
    // _DEBUG_SEE_PROP_SET_VALUE = 0
    // _DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 0

    // ControlKey ctrl_d (f, DJN_Key_D)

	// ctrl_d.press -> (root) {
	// 	if (_DEBUG_SEE_ACTIVATION_SEQUENCE_2) {
	// 		_DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 0
	// 		_DEBUG_SEE_PROP_SET_VALUE = 0
	// 	}
	// 	else {
	// 		_DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 1
	// 		_DEBUG_SEE_PROP_SET_VALUE = 1
	// 	}
	// }

    List models                         // (Ordered) list of all models
    ProcessCollector displayed_models   // (Unordered) collection of the displayed models

    // TextPrinter tp
    // displayed_models.size + " displayed models among the " + models.size + " persons in the full list" => tp.input

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

            ListBox L ()                    // [7GUIs] L presents a view of the data in the database that consists of a list of names.
            
            // FIXME: does NOT work
            // We want a minimum size for the listbox even if the list box is empty
            // L.preferred_width = 250
            // 250 =: L.preferred_width
            // L.preferred_height = 250
            // 250 =: L.preferred_height

            // L.min_width = 250
            // 250 =: L.min_width
            // L.min_height = 250
            // 250 =: L.min_height

            VBox props {
                HBox name {
                    Label L_name("Name:")       // [7GUIs] a pair of textfields Tname and Tsurname,
                    L_name.preferred_width = 60
                    UITextField T_name          // [7GUIs] [with a label],
                }
                HBox surname {
                    Label L_surname("Surname:") // [7GUIs] a pair of textfields Tname and Tsurname,
                    L_surname.preferred_width = 60
                    UITextField T_surname       // [7GUIs] [with a label]
                }
            }
            props.v_alignment = 0
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

    // References on name & surname of the model of the selected item
    DerefString name_of_selected_item (L.selected_item, "model/name", DJNN_GET_ON_CHANGE)
    DerefString surname_of_selected_item (L.selected_item, "model/surname", DJNN_GET_ON_CHANGE)

    // By default, all buttons are disabled
    |-> BC.disable, BU.disable, BD.disable

    models.$added -> na_model_added_to_all_models:(root) {  // Called when a model is created & added to the ordered list of all models
        model = getRef (&root.models.$added)
        if (&model != null) {
            // print ("The model '" + model.surname + "' has been added to the list of all " + root.models.size + " models")
            string prefix = getString (root.T_prefix.field.content.text)
            
            // Add to the list of displayed_models if the prefix is empty OR if the surname starts with it
            if ( (prefix == "") || starts_with (getString (model.surname), prefix) ) {
                setRef (root.displayed_models.add, model)        
            }
        }
    }

    displayed_models.add -> na_model_added_to_displayed_models:(root) {     // Called when a model is added to the unordered list of displayed models (its view should be display)
        model = getRef (&root.displayed_models.add)
        if (&model != null) {
            // Create a view corresponding to the model to display
            // & insert it in the list box at the right index (to correspond to the ordered list with all models)
            insert_view_for_model_to_display (model, root.models, root.L)
        }
    }
    na_model_added_to_displayed_models -> root.L.pack   // Update layout

    NativeAction na_model_removed_from_displayed_models (action_model_removed_from_displayed_models, root, 1)
    displayed_models.rm -> na_model_removed_from_displayed_models           // Called when a model is removed from the unordered list of displayed models (its view should be hide)

    (L.selected_item.is_null == 0) && L.selected_item -> {      // [7GUIs] At most one entry can be selected in L at a time.
        name_of_selected_item.value =: root.T_name.init_text
        surname_of_selected_item.value =: root.T_surname.init_text
    }
    L.selected_item.is_null.true -> root.T_name.clear, root.T_surname.clear     // NO selection -> clear text fields

    NativeAction na_filter (action_filter, root, 1)     // [7GUIs] By entering a string into Tprefix the user can filter the names whose surname start with the entered prefix
    T_prefix.field.content.text -> na_filter            // [7GUIs] —this should happen immediately without having to submit the prefix with enter.

    // Flag indicating whether a value is missing: either the name or the surname
    Bool is_missing_value (true)
    
    // Allow to enable/disable buttons during editing (don't wait to validate a TextField)
    // (getString (T_name.text) == "") || (getString (T_surname.text) == "") => is_missing_value
    (getString (T_name.field.content.text) == "") || (getString (T_surname.field.content.text) == "") => is_missing_value
    
    is_missing_value.false -> BC.enable
    is_missing_value.true -> BC.disable

    L.selected_item.is_null.false -> BU.enable, BD.enable       // [7GUIs] BU and BD are enabled if an entry in L is selected.
    L.selected_item.is_null.true -> BU.disable, BD.disable      // [7GUIs] BU and BD are enabled if an entry in L is selected.

    BC.click -> na_create_person:(root) {               // [7GUIs] Clicking BC will append the resulting name from concatenating the strings in Tname and Tsurname to L.
        addChildrenTo root.models {
            PersonModel _ (getString (root.T_name.text), getString (root.T_surname.text))
        }
    }
    na_create_person -> root.T_name.clear, root.T_surname.clear

    T_name.next -> T_surname.activate       // Key "tab" allows to set the focus to next text field (T_name -> T_surname)
    T_surname.next -> BC.select             // Key "tab" allows to set the focus to next button (T_surname -> BC)

    BU.click -> {       // [7GUIs] In contrast to BC, BU will not append the resulting name but instead replace the selected entry with the new name.
        root.T_name.text =?: name_of_selected_item.value
        root.T_surname.text =?: surname_of_selected_item.value
    }

    NativeAction na_delete_selected_person (action_delete_selected_person, root, 1)
    // BD.click -> L.reset_selection               // drawback: we can't do that because we use the ref on selected_item in the native action "delete_selected_person"
    BD.click -> na_delete_selected_person       // [7GUIs] BD will remove the selected entry.
    na_delete_selected_person -> root.L.pack    // Update layout after deletion


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
// [7GUIs] BU and BD are enabled if an entry in L is selected.
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

