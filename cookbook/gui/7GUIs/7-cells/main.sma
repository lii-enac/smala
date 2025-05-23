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
    djnnstl::string
    ascii_to_string_from(int c, const djnnstl::string& from_char)
    {
        return djnnstl::string("") + (char)(c + from_char[0]);
    }
%}

_main_
Component root {
    Frame f ("7GUIs Cells UNFINISHED", 0, 0, 600, 600)  // [7GUIs] The task is to create a simple but usable spreadsheet application.
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

    Text truc(0,10,"truc") // test Z-Order

    // TextPrinter tp
    // "" + edit.tf.x + " " + edit.tf.y =:> tp.input

    //ZOrder z (-1)
    for (int row_ = 0; row_ < 10; row_++) {
        for (int col_ = 0; col_ < 10; col_++) {
            Component cell {
                Int row(row_) // retain row
                Int col(col_) // retain col
                String formula("formula") // if it starts with an '='
                String value("")          // either a value or the result of the computation of the formula

                NoFill _
                PickFill _
                OutlineColor _(#222222)
                Rectangle bg((col+1)*100, (row+1)*20, 100, 20) // this will receive mouse events

                FillColor _(#FFFFFF)
                Text t((col+1)*100, (row+1)*20+20, "")

                Switch mode(displaying) {
                    Component displaying {
                        value =:> t.text
                        100 =: bg.width
                    }
                    Component editing {
                        "" =: t.text
                        0 =: bg.width // z-order does not seem to work between UITextField and Rectangle, the rectangle prevents editing
                        
                        edit.tf.text -> yop: { // whenever the text has changed
                            // update the formula and the value (TODO this should depend on the presence of '=')
                            edit.tf.text =: formula
                            edit.tf.text =: value
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
                    formula =: edit.tf.text
                    "editing" =: mode.state
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
