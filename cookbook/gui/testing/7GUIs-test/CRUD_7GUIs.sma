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

    Rectangle r (0, 0, 100, 100)

    // List of persons
    PersonsList persons ()

}
