use core
use base
use gui

import gui.widgets.StandAloneUITextField

_native_code_
%{
#include <assert.h>
#include "core/utils/iostream.h"
#include "core/utils/error.h"
#include "utils/debug.h"


#include "base/process_handler.h"

#include "core/utils/getset.h"

djnnstl::string
ascii_to_string_from(int c, const djnnstl::string& from_char)
{
    return djnnstl::string("") + (char)(c + from_char[0]);
}

void
cpp_parse_formula (Process* c)
{
    //std::cerr << c->get_debug_name () << __FL__;
    auto * spreadsheet = reinterpret_cast<Process*>(get_native_user_data (c));

    GET_CHILD(RefProperty, spreadsheet, _current_cell);
    assert (_current_cell);
    auto * cell = _current_cell->get_value();
    assert (cell);
    GET_CHILD(TextProperty, cell, formula);
    assert (formula);

    //debug
    // std::cerr << cell<< __FL__ << std::endl;
    // std::cerr
    //     //<< formula->get_debug_name () << " "
    //     << formula->get_value() << __FL__ << std::endl;
    
    GET_CHILD(ProcessCollector, cell, input_cells);
    assert (input_cells);
    input_cells->remove_all ();
    GET_CHILD(Component, cell, bindings);
    assert (bindings);
    bindings->clean_up_content ();

    const djnnstl::string& f = formula->get_value();
    if (f.find("=")==0) { // TODO: Use a regex if we want to support flexible formula parsing
        // it's a formula
        if (f.find("sum(",1)==1) {

            //debug
            //std::cerr << " SUM DETECTED: " << f << std::endl;

            djnnstl::string::size_type start = 5;
            auto end = f.find(")", start);
            //std::cerr << start << " " << end << __FL__;
            if (end==djnnstl::string::npos) {
                std::cerr << "no ')' found" << __FL__;
                return;
            }
            auto col = f.find(":", start);
            //std::cerr << start << " " << col << " " << end << __FL__;
            if (col==djnnstl::string::npos || col>end) {
                std::cerr << "no ':' found inside range expression" << __FL__;
                return;
            }

            //std::cerr << start << " " << col << " " << end << __FL__;
            auto start_cell_id = f.substr(start, col-start);
            auto end_cell_id = f.substr(col+1, end-(col+1));
            //std::cerr << start_cell_id << " " << end_cell_id << __FL__;
            auto start_cell_id_col_str = start_cell_id[0]-'A';
            auto start_cell_id_row_str = start_cell_id[1]-'0';
            auto end_cell_id_col_str = end_cell_id[0]-'A';
            auto end_cell_id_row_str = end_cell_id[1]-'0';

            GET_CHILD(Component, spreadsheet, grid);
            GET_CHILD(List, grid, cells);
            assert (cells);
            for (auto &c: cells->children()) {
                GET_CHILD_VALUE(col, Int, c, col);
                GET_CHILD_VALUE(row, Int, c, row);
                if (
                    start_cell_id_col_str <= col &&
                    end_cell_id_col_str   >= col &&
                    start_cell_id_row_str <= row &&
                    end_cell_id_row_str   >= row
                    )
                {
                    //debug std::cerr << "new Binding " << (char)(col+'A') << " " << row << " " << c << std::endl;
                    input_cells->add_one(c);
                    new Binding(cell->find_child("bindings"), "b_to_formula", c->find_child("compute_formula"), cell->find_child("compute_formula"));
                }
            }
        }
    } else {
        GET_CHILD(TextProperty, cell, value);
        assert (value);
        value->set_value (f, true);
    }
}

void
cpp_compute_formula (Process* c)
{
    //std::cerr << "cpp_compute_formula " << c->get_debug_name () << std::endl;

    auto * cell = reinterpret_cast<Process*>(get_native_user_data (c));
    assert (cell);
    GET_CHILD(TextProperty, cell, formula);
    assert (formula);

    //std::cerr << formula->get_value () << std::endl;

    // MP  necesaaire ??
    if (formula->get_value().empty()) return;
    if (formula->get_value()[0]!='=') return; 

    GET_CHILD(ProcessCollector, cell, input_cells);
    assert (input_cells);

    int res = 0;
    for (auto* p : input_cells->get_list()) {
        GET_CHILD_VALUE(in_val, Text, p, value);
        //std::cerr << in_val << __FL__;
        if (!in_val.empty())
            res += atoi(in_val.c_str());
    }

    // std::cerr
    //     << dynamic_cast<TextProperty*>(cell->find_child("col_string"))->get_value()
    //     << dynamic_cast<IntProperty*>(cell->find_child("row"))->get_value()
    //     << " " << res
    //     << __FL__;

    GET_CHILD(TextProperty, cell, value);
    value->set_value(res, true);
}

%}

_define_
Spreadsheet (Process root_, int row_, int col_, int tx_, int ty_)
{
    root aka root_

    //------- settings ------
    Translation t (tx_, ty_)
    Component settings {
        FillColor grid_color (#222222)
        FillColor cell_color (#F8F4F1)  // #F8F4F1
        FillColor edit_color (#E67E85)  // #E67E85
    }
    //------------------------
    
    //----- MAGIK NUMBERS ----
    int cell_width = 100
    int cell_height = 26
    int default_text_spacing = 3
    int start_grid_x = 30
    int start_grid_y = 18
    int cell_col_header_height = start_grid_y
    //------------------------

    // col header A B C...
    Component _ {
        for (int col=0; col<col_; col++) {
            FillColor _ (#535353) 
            NoOutline _
            Rectangle _ ((col* cell_width)+start_grid_x, 0, cell_width, cell_col_header_height, 0, 0)
            FillColor _ (White)
            Text _ ((col* cell_width)+start_grid_x+default_text_spacing, cell_col_header_height-default_text_spacing, ascii_to_string_from(col, "A"))
            OutlineColor oc (Black)
            settings.grid_color.value =: oc.value
            Line _ (((col+1) * cell_width)+start_grid_x, 0, ((col+1)* cell_width)+start_grid_x, cell_col_header_height)
        }
    }

    // row header 0 1 2...
    Component _ {
        FillColor _(White)
        for (int row=0; row < row_; row++) {
            Text _(default_text_spacing, (row+1)*cell_height+start_grid_y-(2*default_text_spacing), to_string(row))
        }
    }

    Ref _current_cell(null)
    DerefInt _row_of_current_cell (_current_cell, "row", DJNN_GET_ON_CHANGE)
    DerefInt _col_of_current_cell (_current_cell, "col", DJNN_GET_ON_CHANGE)
    DerefString _formula_of_current_cell (_current_cell, "formula", DJNN_GET_ON_CHANGE)
    DerefString _value_of_current_cell (_current_cell, "value", DJNN_GET_ON_CHANGE)

    Spike reset_box_edit
    Spike toto

    Component grid {
        Translation t (start_grid_x, start_grid_y)
        List cells 
        addChildrenTo cells
        {
            for (int irow = 0; irow < row_; irow++) {
                for (int icol = 0; icol < col_; icol++) {
                    Component cell { // FIXME Adding to a List item will lose is name
                        Int row(irow) // retain row
                        Int col(icol) // retain col
                        String col_string(ascii_to_string_from(icol, "A"))
                        NativeAction parse_formula (cpp_parse_formula, this, 1)
                        NativeAction compute_formula (cpp_compute_formula, cell, 1)

                        String formula("") // if it starts with an '='
                        String value("")      // either a value or the result of the computation of the formula

                        // ------  DEBUG -------
                        // B2 = 90      C2 = 45
                        // B3 = 45      C3 = 0
                        // B4 = 5       C4 = 40
                        // =sum(B2:C4)  == 225
                        if ((col == 1) && (row == 2)){ "90" =: value }  // B2
                        if ((col == 1) && (row == 3)){ "45" =: value }  // B3
                        if ((col == 1) && (row == 4)){ "5" =: value }   // B4
                        if ((col == 2) && (row == 2)){ "45" =: value }  // C2
                        if ((col == 2) && (row == 3)){ "0" =: value }   // C3
                        if ((col == 2) && (row == 4)){ "40" =: value }  // C4
                        // -------------------------


                        //-- DEBUG
                        FillColor fc (Blue)
                        settings.cell_color.value =:> fc.value
                        // NoOutline _

                        // NoFill _
                        // PickFill _
                        OutlineColor oc (Blue)
                        settings.grid_color.value =:> oc.value
                        Rectangle bg((col)*cell_width, (row)*cell_height, cell_width, cell_height) // bg will receive mouse events

                        FillColor _(Black)
                        Text t ((col*cell_width)+default_text_spacing,(row*cell_height)+cell_height-(2*default_text_spacing), "")
                        value =:> t.text

                        bg.press -> { // move the TextField on top of the cell
                            cell =: _current_cell 
                        }

                        Component bindings // this is where we store bindings from other cells e.g. sum(A3:A8), the bindings from A3 to A8
                        ProcessCollector input_cells // this is where we store bindings from other cells e.g. sum(A3:A8), the bindings from A3 to A8

                        formula -> parse_formula
                        parse_formula -> compute_formula
                        //compute_formula ~> value // not sure of this one...

                        compute_formula -> reset_box_edit
                    }
                }
            }
        }
        Component box_edit {
            Translation t (-100,-100) // to control the TextField position
            _col_of_current_cell.value * cell_width => t.tx
            _row_of_current_cell.value * cell_height => t.ty
            FillColor edit_bg_color (White)
            settings.cell_color.value =:> edit_bg_color.value
            OutlineColor oc (Red)
            settings.edit_color.value =:> oc.value
            OutlineWidth ow (2)
            Rectangle _ (0, 0, cell_width, cell_height)
            FillColor _ (Black)
            StandAloneUITextField tf ($ow.width, default_text_spacing, cell_width - default_text_spacing - $ow.width , cell_height) // a TextField we will move on top of cells to edit them
            edit_bg_color.value =:> tf.bg_color.value, tf.bg_ol_color.value

            //toto -> tf.field.press // TODO : NE s'ACTIVE qu'une fois sur deux ??
             
            _formula_of_current_cell.value =:> tf.init_text

            tf.validate -> post_validate : (this) {
                string str = getString (this.grid.box_edit.tf.text)
                this._formula_of_current_cell.value = str
            }

            reset_box_edit -> (this) {  // TODO  on reset_box_edit ??
                //setRef (this._current_cell, nullptr) ///  ou apres le parse_formula  // probleme de VD !! as usual 
                this.grid.box_edit.t.tx = -100
                this.grid.box_edit.t.ty = -100
            }
        }
    }

    // Component grid_gfx { // better draw outline in cell ?
    //     Translation t (start_grid_x, start_grid_y)
    //     OutlineColor oc (Black)
    //     settings.grid_color.value =: oc.value
    //     for (int row=0; row < row_; row++) {      
    //         Line _ (0, (row+1)* cell_height, (row_ * cell_width), (row+1)* cell_height)
    //     }
    //     for (int col=0; col < col_; col++) {      
    //         Line _ (((col+1) * cell_width), 0, ((col+1)* cell_width), (col_ * cell_height))
    //     }
    // } 

    

    // Debug
    Component debug {
        Translation t (0, row_ * cell_height + 3* cell_height)
        FillColor _ (White)
        Text _ (default_text_spacing, 0, "---- DEBUG current cell ---- ")
        Text name (default_text_spacing, cell_height, "Name :")
        Text formula (default_text_spacing, 2*cell_height, "Formula: ")
        Text value (default_text_spacing, 3*cell_height, "Value:")
        "Name: (" + _row_of_current_cell.value + ", " + _col_of_current_cell.value + ")" =:> name.text
        "Formula: " + _formula_of_current_cell.value =:> formula.text
        "Value: " + _value_of_current_cell.value =:> value.text
    }



    // moving edit box
    // Component edit {
    //     ZOrder z (11)
    //     Translation tr(0,0) // to control the TextField position
    //     HBox h {
    //         UITextField tf // a TextField we will move on top of cells to edit them
    //     }
    //     tf aka h.tf

    //     h.v_alignment = 0 // top
    //     h.h_alignment = 0 // left
        
    //     tf.text_color = #FF0000
    //     tf.preferred_width = 100
    //     "toto" =: edit.tf.text
    // }

    // Text _(0,10,"z test") // test Z-Order

    // NativeAction parse_formula (cpp_parse_formula, this, 1)

    // TextPrinter tp
    // // "" + edit.tf.x + " " + edit.tf.y =:> tp.input

    //Ref current_cell(null)
    

    // //ZOrder z (-1)
    //Component cells
    //List cells // prefer a list so as to have access to all of its children
    // addChildrenTo cells
    // {
    //     for (int row_ = 0; row_ < 10; row_++) {
    //         for (int col_ = 0; col_ < 10; col_++) {
    //             //string name = "cells" + valueOf(row_)
    //             Component cell { // FIXME this will replace previous "cell" in the component symtable. How to generate a name? change a name?
    //                 Int row(row_) // retain row
    //                 Int col(col_) // retain col
    //                 String col_string(ascii_to_string_from(col_, "A"))

    //                 String formula("") // if it starts with an '='
    //                 String value("")      // either a value or the result of the computation of the formula

    //                 NoFill _
    //                 PickFill _
    //                 OutlineColor _(#222222)
    //                 Rectangle bg((col+1)*100, (row+1)*20, 100, 20) // this will receive mouse events

    //                 FillColor _(#FFFFFF)
    //                 Text t((col+1)*100, (row+1)*20+20, "")

    //                 Component bindings // this is where we store bindings from other cells e.g. sum(A3:A8), the bindings from A3 to A8
    //                 ProcessCollector input_cells // this is where we store bindings from other cells e.g. sum(A3:A8), the bindings from A3 to A8

    //                 //formula -> parse_formula
    //                 //NativeAction compute_formula (cpp_compute_formula, cell, 1)
    //                 //parse_formula -> compute_formula
    //                 //compute_formula ~> value // not sure of this one...

    //                 // input_cells -> (cell) {
    //                 //     Binding (cell.bindings, "", )
    //                 // }

    //                 Switch mode(displaying) {
    //                     Component displaying {
    //                         value =:> t.text
    //                         100 =: bg.width
    //                     }
    //                     Component editing {
    //                         // "" =: t.text
    //                         // 0 =: bg.width // FIXME z-order does not seem to work between UITextField and Rectangle, the rectangle prevents editing

    //                         // edit.tf.text -> yop: { // whenever the text has changed
    //                         //     // update the formula and the value (TODO this should depend on the presence of '=')
    //                         //     edit.tf.text =: formula
    //                         //     //edit.tf.text =: value
    //                         //     // move back the TextField
    //                         //     0 =: edit.tr.tx
    //                         //     0 =: edit.tr.ty

    //                         //     "displaying" =: mode.state
    //                         // }
    //                         // yop -> edit.tf.clear
    //                     }
    //                 }

    //                 // bg.press -> { // move the TextField on top of the cell
    //                 //     (col+1)*100 =: edit.tr.tx
    //                 //     (row+1)*20  =: edit.tr.ty
    //                 //     formula =: edit.tf.text // FIXME does not work :-/
    //                 //     "editing" =: mode.state
    //                 //     cell =: current_cell // this is how to set a ref in an assignment sequence...
    //                 // }
    //             }
    //         }   
    //     }
    // }
} 