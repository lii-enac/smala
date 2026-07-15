/*
*  Smala cookbook list_viewer
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2022)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*
*/
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