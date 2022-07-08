use core
use base

import gui.widgets.ItemView

_define_
SimpleFlightView () inherits ItemView ()
{
    FillColor _ (164, 197, 255)
    OutlineColor _ (White)
    OutlineWidth _ (2)
    Rectangle bkg (0, 0, 100, 30, 5, 5)
    this.width = 100
    this.height = 30
    FillColor _ (White)
    Text flight_name (5, 15, "")
    DerefString model_flight_name (this.model, "flight", DJNN_GET_ON_CHANGE)
    model_flight_name.value =:> flight_name.text
}