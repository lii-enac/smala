use core
use base
use gui

/*
 * Converts from the local coordinates system of inShape to the local coordinates system of outShape
 * (both shapes must be graphical shapes, ie must have a transformation matrix)
 */
_define_
LocalToLocal (Component inShape, Component outShape) {

    LocalToScreen l2s (inShape)
    ScreenToLocal s2l (outShape)
    l2s.outX => s2l.inX
    l2s.outY => s2l.inY

    /* --- input interface: coordinates in inShape local coordinates system --- */
    inX aka l2s.inX
    inY aka l2s.inY
    /* -- output interface: coordinates in outShape local coordinates system -- */
    outX aka s2l.outX
    outY aka s2l.outY
    /* -- end interface -- */
}