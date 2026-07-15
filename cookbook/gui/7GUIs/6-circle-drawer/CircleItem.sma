/*
*  Smala cookbook 6-circle-drawer
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2025-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use gui


_define_
CircleItem (Process root_, int x, int y)
{
    FillColor fc(200,200,200)               // [7GUIs] (the color gray)
    NoFill nf                               // [7GUIs] ...an unfilled...
    PickFill _
    OutlineColor oc(255,255,255)
    Circle c(x, y, 10)                      // [7GUIs] ...circle with a fixed diameter whose center is the left-clicked point.
    Spike deselect

    c.enter ->! nf                          // [7GUIs] The circle nearest to the mouse pointer such that the distance from its center to the pointer is less than its radius, if it exists, is filled with the color gray.
    // c.leave -> nf // NOT that simple !
    deselect -> nf

    root aka root_

    c.right.press -> {                            // [7GUIs] Right-clicking C will make a popup menu appear with one entry “Adjust diameter..”.
        c.press.x =: root.menu.x
        c.press.y =: root.menu.y
    }
    c.right.press -> root.menu.unfold

    // Enter deselects the previously selected one (if any) and selects the current item and 
    c.enter -> (this) {
        old_selected_item = getRef(&this.root._selected_item_ref)
        if (&old_selected_item != null && &old_selected_item != &this) {
            activate (old_selected_item.deselect)
        }
        graph_exec ()
        setRef(this.root._selected_item_ref, this)
    }
}  