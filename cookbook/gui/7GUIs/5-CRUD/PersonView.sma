use core
use base

import ListBoxItem

_define_
PersonView (Process _model, int _index) inherits ListBoxItem (_index)
{
    model aka _model

    // Link properties of the model and set the content of the (inherited) label
    _model.surname + ", " + _model.name =:> this.text

    // TextPrinter tp
    // "View of person [" + this.index + "]: " + _model.surname + ", " + _model.name =:> tp.input
}