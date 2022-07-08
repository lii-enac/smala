use core
use base

import gui.widgets.ItemView

_define_
FullFlightView () inherits ItemView ()
{
    FillColor _ (#FFE1B9)
    OutlineColor _ (Black)
    Rectangle bkg (0, 0, 200, 50, 0, 0)
    this.width = 100
    this.height = 50
    FillColor _ (Black)
    Text flight_name (5, 15, "")
    Text flight_level (5, 30, "")
    Text flight_speed (5, 45, "")
    
    DerefString model_flight_name (this.model, "flight", DJNN_GET_ON_CHANGE)
    DerefInt model_flight_level (this.model, "FL", DJNN_GET_ON_CHANGE)
    DerefInt model_flight_speed (this.model, "speed", DJNN_GET_ON_CHANGE)
    model_flight_name.value =:> flight_name.text
    model_flight_level.value =:> flight_level.text
    model_flight_speed.value =:> flight_speed.text
}