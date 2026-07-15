/*
*  Smala cookbook 7GUIs-test
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2025-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
// [7GUIs] CRUD
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#CRUD
// [7GUIs] Challenges: separating the domain and presentation logic, managing mutation, building a non-trivial layout.

use core
use base
use display
use gui

// import gui.widgets.PushButton
// import gui.widgets.UITextField
// import gui.widgets.VBox
// import gui.widgets.HBox
// import gui.widgets.Label
// import gui.widgets.ListBox

// import ../../7GUIs/5-CRUD/PersonModel
// import ../../7GUIs/5-CRUD/PersonView
import ../../7GUIs/5-CRUD/PersonsList




_define_
CRUD_7GUIs () {
    Frame f ("7GUIs CRUD", 1100, 0, 380, 280)  // [7GUIs] The task is to build a frame containing the following elements:
    // f.close ->! mainloop

    // List of persons
    PersonsList persons ()

}
