/*
*  Smala Library
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017-2026)
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

import gui.widgets.IWidget

_define_
ProgressBar (int _init_val) inherits IWidget () {
    BoundedValue bv (0, 100, _init_val)
    min aka bv.min
    max aka bv.max
    output aka bv.result

    Double value (_init_val)
    bv.result => value
    
    this.min_width = 100
    this.preferred_width = 100

    int default_height  = 15
    this.min_height = default_height
    this.preferred_height = default_height

    OutlineColor outln_c (#535353)
    FillColor bg_color (#DDDDDD)
    Rectangle bg (0, 0, 100, default_height, 3, 3)

    FillColor filled_color (90, 190, 250)
    Rectangle r_filled (0, 0, 50, default_height, 3, 3)
    
    (value-min)/(max-min) * bg.width =:> r_filled.width

    // TextPrinter tp
    // "Progress [" + min + " - " + max + "]: " + value =:> tp.input
}