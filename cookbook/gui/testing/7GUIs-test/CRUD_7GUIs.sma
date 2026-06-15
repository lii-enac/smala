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
import gui.widgets.Label
import gui.widgets.ListBox

// import ../../7GUIs/5-CRUD/PersonModel
// import 7GUIs.5-CRUD.PersonModel
import CRUD.PersonModel
// import ../../7GUIs/5-CRUD/PersonView
// import 7GUIs.5-CRUD.PersonView
import CRUD.PersonView

_native_code_
%{
    // #include "../../7GUIs/5-CRUD/PersonModel.h"
    // #include "../../7GUIs/5-CRUD/PersonView.h"

    #include "core/property/text_property.h"
    #include <algorithm>

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

        for view : self.LB.items {
            if (&view.model == &model) {            // We found the view of the removed model
                if (view.is_selected) {
                    notify self.LB.reset_selection       // Reset the selection // drawback: not visible in the main code
                    // activate (self.LB.reset_selection)   // Replace notify by activate (...) ?
                }

                remove_one (self.LB.items, view)     // Remove from the ProcessCollector "list_box.items"
                remove view from self.LB             // Remove from children list of the list box

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
    selected_item = getRef (self.LB.selected_item)
    if (&selected_item != null) {
        // print ("Delete view & model of " + selected_item.text)

        notify self.LB.reset_selection                   // Reset the selection // drawback: not visible in the main code
        // activate (self.LB.reset_selection)               // Replace notify by activate (...) ?
        
        Process model = find (selected_item, "model")   // Get the corresponding model

        remove_one (self.LB.items, selected_item)        // Remove view from collection "items"
        remove selected_item from self.LB                // Remove view from children list

        if (&model != null) {
            if (contains (self.displayed_models, model)) {  // Could be filtered out
                // setRef (self.displayed_models.rm, model)
                // graph_exec ()
                remove_one (self.displayed_models, model)   // We have to remove from the list of displayed_models immediately !
            }

            delete selected_item            // Delete view

            remove model from self.models   // Remove from list of all models
            delete model                    // Delete the model
        }
    }
}


_define_
CRUD_7GUIs () {

    Frame f ("7GUIs CRUD", 0, 0, 600, 600)  // [7GUIs] The task is to build a frame containing the following elements:
    // f.close ->! mainloop

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

            ListBox LB ()                    // [7GUIs] L presents a view of the data in the database that consists of a list of names.
            
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
    LB aka b_all.data.LB
    T_name aka b_all.data.props.name.T_name
    T_surname aka b_all.data.props.surname.T_surname
    BC aka b_all.buttons.BC
    BU aka b_all.buttons.BU
    BD aka b_all.buttons.BD

    // References on name & surname of the model of the selected item
    DerefString name_of_selected_item (LB.selected_item, "model/name", DJNN_GET_ON_CHANGE)
    DerefString surname_of_selected_item (LB.selected_item, "model/surname", DJNN_GET_ON_CHANGE)

    // By default, all buttons are disabled
    |-> BC.disable, BU.disable, BD.disable

    models.$added -> na_model_added_to_all_models:(this) {  // Called when a model is created & added to the ordered list of all models
        model = getRef (&this.models.$added)
        if (&model != null) {
            // print ("The model '" + model.surname + "' has been added to the list of all " + this.models.size + " models")
            string prefix = getString (this.T_prefix.field.content.text)
            
            // Add to the list of displayed_models if the prefix is empty OR if the surname starts with it
            if ( (prefix == "") || starts_with (getString (model.surname), prefix) ) {
                setRef (this.displayed_models.add, model)        
            }
        }
    }

    displayed_models.add -> na_model_added_to_displayed_models:(this) {     // Called when a model is added to the unordered list of displayed models (its view should be display)
        model = getRef (&this.displayed_models.add)
        if (&model != null) {
            // Create a view corresponding to the model to display
            // & insert it in the list box at the right index (to correspond to the ordered list with all models)
            insert_view_for_model_to_display (model, this.models, this.LB)
        }
    }
    na_model_added_to_displayed_models -> LB.pack   // Update layout

    NativeAction na_model_removed_from_displayed_models (action_model_removed_from_displayed_models, this, 1)
    displayed_models.rm -> na_model_removed_from_displayed_models           // Called when a model is removed from the unordered list of displayed models (its view should be hide)

    (LB.selected_item.is_null == 0) && LB.selected_item -> {      // [7GUIs] At most one entry can be selected in L at a time.
        name_of_selected_item.value =: T_name.init_text
        surname_of_selected_item.value =: T_surname.init_text
    }
    LB.selected_item.is_null.true -> T_name.clear, T_surname.clear     // NO selection -> clear text fields

    NativeAction na_filter (action_filter, this, 1)     // [7GUIs] By entering a string into Tprefix the user can filter the names whose surname start with the entered prefix
    T_prefix.field.content.text -> na_filter            // [7GUIs] —this should happen immediately without having to submit the prefix with enter.

    // Flag indicating whether a value is missing: either the name or the surname
    Bool is_missing_value (true)
    
    // Allow to enable/disable buttons during editing (don't wait to validate a TextField)
    // (getString (T_name.text) == "") || (getString (T_surname.text) == "") => is_missing_value
    (getString (T_name.field.content.text) == "") || (getString (T_surname.field.content.text) == "") => is_missing_value
    
    is_missing_value.false -> BC.enable
    is_missing_value.true -> BC.disable

    LB.selected_item.is_null.false -> BU.enable, BD.enable       // [7GUIs] BU and BD are enabled if an entry in L is selected.
    LB.selected_item.is_null.true -> BU.disable, BD.disable      // [7GUIs] BU and BD are enabled if an entry in L is selected.

    BC.click -> na_create_person:(this) {               // [7GUIs] Clicking BC will append the resulting name from concatenating the strings in Tname and Tsurname to LB.
        addChildrenTo this.models {
            PersonModel _ (getString (this.T_name.text), getString (this.T_surname.text))
        }
    }
    na_create_person -> T_name.clear, T_surname.clear

    T_name.next -> T_surname.activate       // Key "tab" allows to set the focus to next text field (T_name -> T_surname)
    T_surname.next -> BC.select             // Key "tab" allows to set the focus to next button (T_surname -> BC)

    BU.click -> {       // [7GUIs] In contrast to BC, BU will not append the resulting name but instead replace the selected entry with the new name.
        T_name.text =?: name_of_selected_item.value
        T_surname.text =?: surname_of_selected_item.value
    }

    NativeAction na_delete_selected_person (action_delete_selected_person, this, 1)
    // BD.click -> LB.reset_selection               // drawback: we can't do that because we use the ref on selected_item in the native action "delete_selected_person"
    BD.click -> na_delete_selected_person       // [7GUIs] BD will remove the selected entry.
    na_delete_selected_person -> LB.pack    // Update layout after deletion

}
