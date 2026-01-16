use core
use base

// import gui.widgets.ItemView
import ListBoxItem

_define_
// PersonView () inherits ItemView ()
PersonView (Process _model, int _index) inherits ListBoxItem (_model, _index)
{
    // FillColor _ (#666666)
    // OutlineColor _ (White)
    // OutlineWidth _ (2)
    
    // Rectangle bkg (0, 0, 100, 30, 5, 5)
    // this.width = 100
    // this.height = 30

    // FillColor _ (White)
    // // Text txt_full_name (5, 15, "")
    // Text txt_surname (5, 15, "")
    // Text txt_name (65, 15, "")

    // // txt_surname.x + txt_surname.width =:> txt_name.x

    // DerefString ref_name (this.model, "name", DJNN_GET_ON_CHANGE)
    // DerefString ref_surname (this.model, "surname", DJNN_GET_ON_CHANGE)
    // // ref_surname.value + ", " + ref_name.value =:> txt_full_name.text
    // ref_surname.value =:> txt_surname.text
    // ref_name.value =:> txt_name.text

    _model.surname + ", " + _model.name =:> this.text
}