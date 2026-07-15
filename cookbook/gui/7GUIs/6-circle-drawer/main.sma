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
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
// [7GUIs] Circle Drawer
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#CRUD
// [7GUIs] Challenges: undo/redo, custom drawing, dialog control.

use core
use base
use display
use gui

import CircleDrawer

_main_
Component root {
    Frame f ("7GUIs Circle Drawer", 300, 300, 400, 460)  // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop
    mouseTracking = 1 // for enter and leave events FIXME should be automatic

    // _DEBUG_SEE_COLOR_PICKING_VIEW = 1

    // Circle drawer with Undo/Redo
    CircleDrawer circle_drawer (f.width, f.height)

}




// [7GUIs]  The task is to build a frame containing an undo and redo button as well as a canvas area underneath.
// [7GUIs]  Left-clicking inside an empty area inside the canvas will create an unfilled circle with a fixed diameter whose center is the left-clicked point.
// [7GUIs]  The circle nearest to the mouse pointer such that the distance from its center to the pointer is less than its radius, if it exists, is filled with the color gray.
// [7GUIs]  The gray circle is the selected circle C. Right-clicking C will make a popup menu appear with one entry “Adjust diameter..”.
// [7GUIs]  Clicking on this entry will open another frame with a slider inside that adjusts the diameter of C.
// [7GUIs]  Changes are applied immediately.
// [7GUIs]  Closing this frame will mark the last diameter as significant for the undo/redo history.
// [7GUIs]  Clicking undo will undo the last significant change (i.e. circle creation or diameter adjustment).
// [7GUIs]  Clicking redo will reapply the last undoed change unless new changes were made by the user in the meantime.
// [7GUIs] 
// [7GUIs] Circle Drawer’s goal is, among other things, to test how good the common challenge of implementing an undo/redo functionality for a GUI application can be solved.
// [7GUIs] In an ideal solution the undo/redo functionality comes for free resp. just comes out as a natural consequence of the language / toolkit / paradigm.
// [7GUIs] Moreover, Circle Drawer tests how dialog control*, i.e. keeping the relevant context between several successive GUI interaction steps, is achieved in the source code.
// [7GUIs] Last but not least, the ease of custom drawing is tested.// [7GUIs] 

// [7GUIs] * Dialog control is explained in more detail in the paper Developing GUI Applications: Architectural Patterns Revisited starting on page seven.
// [7GUIs] The term describes the challenge of retaining context between successive GUI operations.