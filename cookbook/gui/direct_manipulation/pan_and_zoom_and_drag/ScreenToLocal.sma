use core
use base
use gui

/*
 * Converts from the screen coordinates to the local coordinates system of a component
 */
_define_
ScreenToLocal (Component c) {

    /* --- input interface ---- */
    Double screenX (0)
    Double screenY (0)
    /* --- output interface --- */
    Double localX (0)
    Double localY (0)
    /* --- end interface ------ */

    // FIXDJNN aka ou pas, dès appel sur c.matrix il faut que djnn la créée
    // Be sure that the component have a matrix/inverted_matrix by activating smthg on press
    besure = find (c.press)

    m aka c.inverted_matrix

    m.m11 * screenX + m.m12 * screenY + m.m14 - c.origin_x => localX
    m.m21 * screenX + m.m22 * screenY + m.m24 - c.origin_x => localY
}