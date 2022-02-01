/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2019)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Stéphane Conversy <stephane.conversy@enac.fr>
 *
 */

use core
use base
use display
use gui

import gui.widgets.WidgetContainer
import gui.widgets.PushButton
import gui.widgets.VBox
import gui.widgets.HBox
import gui.widgets.Slider
import gui.widgets.Label

_define_
HMI ()
{
    Frame f ("my frame", 0, 0, 300, 300)
    Exit ex (0, 1)
    f.close -> ex
    f.background_color.r = 200
    f.background_color.g = 200
    f.background_color.b = 200

    WidgetContainer wc (f) {
        VBox vbox (0)

        Label l (wc, "fahrenheit:", 0, 0)
        Slider sf (wc, 0, 0)
        "fahrenheit: " + sf.value =:> l.text
        
        Label l2 (wc, "celsius:", 0, 0)
        Slider sc (wc, 0, 0)
        "celsius: " + sc.value =:> l2.text

        addChildrenTo vbox.items {l, sf, l2, sc}
    }
    sf aka wc.vbox.items.[2]
    sc aka wc.vbox.items.[4]
}
