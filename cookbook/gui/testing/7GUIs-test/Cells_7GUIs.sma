use core
use base
use display
use gui

import ../../7GUIs/7-cells/spreadsheet/Spreadsheet

_define_
Cells_7GUIs () {
    Frame f("7GUIs Cells", 620, 360, 500, 500)                    // [7GUIs] The task is to build a frame containing...
    //f.close ->! mainloop

    // [7GUIs] The spreadsheet should be scrollable.
    // [7GUIs] The rows should be numbered from 0 to 99 and the columns from A to Z.

    Spreadsheet sheet (10, 10, 0, 50)
}