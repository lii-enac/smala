// [7GUIs] Cells
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#cells
// [7GUIs] Challenges: change propagation, widget customization, implementing a more authentic/involved GUI application.

use core
use base
use display
use gui

import gui.widgets.UITextField
import gui.widgets.HBox

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
    auto * root = reinterpret_cast<Process*>(get_native_user_data (c));
    GET_CHILD(RefProperty, root, current_cell);
    assert (current_cell);
    
    auto * cell = current_cell->get_value();
    assert (cell);
    GET_CHILD(TextProperty, cell, formula);
    assert (formula);
    //std::cerr << cell<< __FL__;
    // std::cerr
    //     //<< formula->get_debug_name () << " "
    //     << formula->get_value() << __FL__;
    
    GET_CHILD(ProcessCollector, cell, input_cells);
    assert (input_cells);
    input_cells->remove_all ();
    // TODO : deletion of Bindings

    const djnnstl::string& f = formula->get_value();
    if (f.find("=")==0) {
        // it's a formula
        if (f.find("sum(",1)==1) {
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

            if (start_cell_id_col_str != end_cell_id_col_str && start_cell_id_col_str != end_cell_id_row_str) {
                std::cerr << "'" << f.substr(start,end-start) << "' is not a range" << __FL__;
            }

            //std::cerr << "'" << f.substr(start,end-start) << "' is a range" << __FL__;

            // TODO
            // collect all cells
            // bind cell change to recomputation
            // recompute according to function

            // GET_CHILD(Component, root, cells);
            // for (auto &cpair: cells->symtable()) { // does not work since all elements are named 'cell'!!
            //     auto &c = cpair.second;
            GET_CHILD(List, root, cells);
            
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
                    // std::cerr << "new Binding " << (char)(col+'A') << " " << row << " " << c << __FL__;
                    input_cells->add_one(c);
                    new Binding(cell->find_child("bindings"), "bb", c->find_child("compute_formula"), cell);
                }
            }
        }
    } else {
        GET_CHILD(TextProperty, cell, value);
        assert (value);
        // std::cerr << f << __FL__;
        value->set_value (f, true);
    }
}

void
cpp_compute_formula (Process* c)
{
    //std::cerr << c->get_debug_name () << __FL__;
    auto * cell = reinterpret_cast<Process*>(get_native_user_data (c));
    assert (cell);

    GET_CHILD(TextProperty, cell, formula);
    assert (formula);

    //std::cerr << formula()->get_value << __FL__;

    if (formula->get_value().empty()) return;
    if (formula->get_value()[0]!='=') return; // hack could be better

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

// _action_
// hook_action_on_die (Process src, Process data)
// {   
//     print ("\n\n helloIvy:  HAHHHAAA I don't WAANNNT to DIE !!!! \n\n")
// }

_main_
Component root {
    Frame f ("7GUIs Cells UNFINISHED", 600, 0, 600, 600)  // [7GUIs] The task is to create a simple but usable spreadsheet application.
    f.close ->! mainloop

    // [7GUIs] The spreadsheet should be scrollable.
    // [7GUIs] The rows should be numbered from 0 to 99 and the columns from A to Z.


    FillColor _(#FFFFFF)

    // col header A B C...
    for (int col=0; col<10; col++) {
        Text _((col+1)*100, 20, ascii_to_string_from(col, "A"))
    }

    // row header 0 1 2...
    for (int row=0; row<10; row++) {
        Text _(0, (row+1)*20+20, to_string(row))
    }

    // moving edit box
    Component edit {
        ZOrder z (11)
        Translation tr(0,0) // to control the TextField position
        HBox h(f) {
            UITextField tf_ // a TextField we will move on top of cells to edit them
            addChildrenTo h.items {
                tf_
            }
        }
        tf aka h.items.[1]

        h.v_alignment = 0 // top
        h.h_alignment = 0 // left
        
        tf.text_color = #FF0000
        tf.preferred_width = 100
        "toto" =: edit.tf.text
    }

    Text _(0,10,"z test") // test Z-Order

    NativeAction parse_formula (cpp_parse_formula, root, 1)

    TextPrinter tp
    // "" + edit.tf.x + " " + edit.tf.y =:> tp.input

    Ref current_cell(null)

    //ZOrder z (-1)
    //Component cells
    List cells // prefer a list so as to have access to all of its children
    addChildrenTo cells
    {
        for (int row_ = 0; row_ < 10; row_++) {
            for (int col_ = 0; col_ < 10; col_++) {
                //string name = "cells" + valueOf(row_)
                Component cell { // FIXME this will replace previous "cell" in the component symtable. How to generate a name? change a name?
                    Int row(row_) // retain row
                    Int col(col_) // retain col
                    String col_string(ascii_to_string_from(col_, "A"))

                    String formula("") // if it starts with an '='
                    String value("")      // either a value or the result of the computation of the formula

                    NoFill _
                    PickFill _
                    OutlineColor _(#222222)
                    Rectangle bg((col+1)*100, (row+1)*20, 100, 20) // this will receive mouse events

                    FillColor _(#FFFFFF)
                    Text t((col+1)*100, (row+1)*20+20, "")

                    Component bindings // this is where we store bindings from other cells e.g. sum(A3:A8), the bindings from A3 to A8
                    ProcessCollector input_cells // this is where we store bindings from other cells e.g. sum(A3:A8), the bindings from A3 to A8

                    formula -> parse_formula
                    NativeAction compute_formula (cpp_compute_formula, cell, 1)
                    parse_formula -> compute_formula
                    compute_formula ~> value // not sure of this one...

                    // input_cells -> (cell) {
                    //     Binding (cell.bindings, "", )
                    // }

                    Switch mode(displaying) {
                        Component displaying {
                            value =:> t.text
                            100 =: bg.width
                        }
                        Component editing {
                            "" =: t.text
                            0 =: bg.width // FIXME z-order does not seem to work between UITextField and Rectangle, the rectangle prevents editing

                            edit.tf.text -> yop: { // whenever the text has changed
                                // update the formula and the value (TODO this should depend on the presence of '=')
                                edit.tf.text =: formula
                                //edit.tf.text =: value
                                // move back the TextField
                                0 =: edit.tr.tx
                                0 =: edit.tr.ty

                                "displaying" =: mode.state
                            }
                            yop -> edit.tf.clear
                        }
                    }

                    bg.press -> { // move the TextField on top of the cell
                        (col+1)*100 =: edit.tr.tx
                        (row+1)*20  =: edit.tr.ty
                        formula =: edit.tf.text // FIXME does not work :-/
                        "editing" =: mode.state
                        cell =: current_cell // this is how to set a ref in an assignment sequence...
                    }
                }
            }   
        }
    }

    
}



// [7GUIs] The task is to create a simple but usable spreadsheet application.
// [7GUIs] The spreadsheet should be scrollable.
// [7GUIs] The rows should be numbered from 0 to 99 and the columns from A to Z.
// [7GUIs] Double-clicking a cell C lets the user change C’s formula.
// [7GUIs] 
// [7GUIs] After having finished editing the formula is parsed and evaluated and its updated value is shown in C.
// [7GUIs] In addition, all cells which depend on C must be reevaluated.
// [7GUIs] This process repeats until there are no more changes in the values of any cell (change propagation).
// [7GUIs] Note that one should not just recompute the value of every cell but only of those cells that depend on another cell’s changed value.
// [7GUIs] If there is an already provided spreadsheet widget it should not be used.
// [7GUIs] Instead, another similar widget (like JTable in Swing) should be customized to become a reusable spreadsheet widget.
// [7GUIs] 
// [7GUIs] Cells is a more authentic and involved task that tests if a particular approach also scales to a somewhat bigger application.
// [7GUIs] The two primary GUI-related challenges are intelligent propagation of changes and widget customization.
// [7GUIs] Admittedly, there is a substantial part that is not necessarily very GUI-related but that is just the nature of a more authentic challenge.
// [7GUIs] 
// [7GUIs] A good solution’s change propagation will not involve much effort and the customization of a widget should not prove too difficult.
// [7GUIs] The domain-specific code is clearly separated from the GUI-specific code. The resulting spreadsheet widget is reusable.
// [7GUIs] 
// [7GUIs] Cells is directly inspired by the SCells spreadsheet example from the book Programming in Scala.
// [7GUIs] Please refer to the book (or the implementations in this repository) for more details
// [7GUIs] especially with respect to the not directly GUI-related concerns like parsing and evaluating formulas
// [7GUIs] and the precise syntax and semantics of the spreadsheet language.
