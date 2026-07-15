/*
*  Smala cookbook 7GUIs-test
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base
use display
use gui

import ../../7GUIs/7-cells/spreadsheet/Spreadsheet

_define_
Cells_7GUIs () {
    Frame f("7GUIs Cells", 510, 360, 970, 500)                    // [7GUIs] The task is to build a frame containing...
    //f.close ->! mainloop

    // [7GUIs] The spreadsheet should be scrollable.
    // [7GUIs] The rows should be numbered from 0 to 99 and the columns from A to Z.

    Spreadsheet sheet (10, 10, 0, 50)
}