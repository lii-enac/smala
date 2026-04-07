// [7GUIs] Cells
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#cells
// [7GUIs] Challenges: change propagation, widget customization, implementing a more authentic/involved GUI application.

use core
use base
use display
use gui

import spreadsheet.Spreadsheet

// _action_
// hook_action_on_die (Process src, Process data)
// {   
//     print ("\n\n helloIvy:  HAHHHAAA I don't WAANNNT to DIE !!!! \n\n")
// }

_main_
Component root {
    Frame f ("7GUIs Cells")  // [7GUIs] The task is to create a simple but usable spreadsheet application.
    f.close ->! mainloop

    // [7GUIs] The spreadsheet should be scrollable.
    // [7GUIs] The rows should be numbered from 0 to 99 and the columns from A to Z.

    Spreadsheet sheet (root, 10, 10, 0, 50)
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
