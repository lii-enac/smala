// [7GUIs] Circle Drawer
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#CRUD
// [7GUIs] Challenges: undo/redo, custom drawing, dialog control.

use core
use base
use display
use gui

import gui.widgets.PushButton
import gui.widgets.HBox
import gui.widgets.DropDownMenu
import gui.widgets.StandAloneSlider

import action.UndoRedoManager
import action.CreateAction
import action.ChangeRadiusAction

import CircleItem

_main_
Component root {
    Frame f ("7GUIs Circle Drawer - DOES NOT WORK YET AS REQUIRED", 0, 0, 600, 600)  // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop
    mouseTracking = 1 // for enter and leave events FIXME should be automatic

    PushButton _undo("Undo")                        // [7GUIs] ...an undo and...
    PushButton _redo("Redo")                        // [7GUIs] ...redo button... 

    HBox hbox(f) {}
    addChildrenTo hbox.items {
        _undo, _redo
    }
    // FIXME now that all widgets have been reparented by addChildrenTo, they are inaccessible :-/
    undo aka hbox.items.[1]
    redo aka hbox.items.[2]

    Component canvas                                // [7GUIs] ...as well as a canvas area underneath.

    DropDownMenu menu (0, -100)                     // [7GUIs] a popup menu...
    addChildrenTo menu.choices {
        String _ ("Adjust diameter...")             // [7GUIs] ... with one entry “Adjust diameter..”.
    }

    StandAloneSlider radius_slider(0,100,200,10)    // [7GUIs] ...a slider inside that adjusts the diameter of C.

    UndoRedoManager undo_redo_manager
    undo.click -> undo_redo_manager.undo            // [7GUIs] Clicking undo will undo the last significant change (i.e. circle creation or diameter adjustment).
    redo.click -> undo_redo_manager.redo            // [7GUIs] Clicking redo will reapply the last undoed change unless new changes were made by the user in the meantime.

    Ref gcircle_ref_(nullptr)

    f.background_rect.press -> (root) {             // [7GUIs] Left-clicking inside an empty area inside the canvas will create...
        addChildrenTo root.canvas {
            // FIXME? cannot create inline component with another, deeper native 
            CircleItem gcircle (root, $root.f.background_rect.press.x, $root.f.background_rect.press.y) // [7GUIs] ...circle with a fixed diameter whose center is the left-clicked point.
            setRef(root.gcircle_ref_, &gcircle)
        }
        notify root.undo_redo_manager.removeActionsStartingFromCurrent // FIXME should be done by undo_redo_manager(?)
        graph_exec () // force sync with notify // FIXME should be done by undo_redo_manager(?)
        Process ca_ = CreateAction (root.undo_redo_manager.actions, "ca_", getRef(root.gcircle_ref_))
    }

    // Bool gcircle_not_null (0)
    // getRef(&gcircle_ref_) != null =:> gcircle_not_null

    // TODO [7GUIs] Clicking on this entry will open another frame with a slider inside that adjusts the diameter of C.
    // TODO [7GUIs] Closing this frame will mark the last diameter as significant for the undo/redo history.

    radius_slider.output -> (root) {                // [7GUIs] Changes are applied immediately. 
        gcircle = getRef(&root.gcircle_ref_)
        if (&gcircle != null) {
            gcircle.c.r = $root.radius_slider.output
        }
    }

    radius_slider.button_fsm.idle -> (root) {
        gcircle = getRef(&root.gcircle_ref_)
        if (&gcircle != null) {
            notify root.undo_redo_manager.removeActionsStartingFromCurrent // FIXME should be done by undo_redo_manager(?)
            graph_exec () // force sync with notify // FIXME should be done by undo_redo_manager(?)
            // FIXME save old radius before changing it with the slider
            Process ca_ = ChangeRadiusAction (root.undo_redo_manager.actions, "cra_", getRef(root.gcircle_ref_), 10, $gcircle.c.r)
        }
    }

    
    
}




// [7GUIs] The task is to build a frame containing an undo and redo button as well as a canvas area underneath.
// [7GUIs] Left-clicking inside an empty area inside the canvas will create an unfilled circle with a fixed diameter whose center is the left-clicked point.
// [7GUIs] The circle nearest to the mouse pointer such that the distance from its center to the pointer is less than its radius, if it exists, is filled with the color gray.
// [7GUIs] The gray circle is the selected circle C. Right-clicking C will make a popup menu appear with one entry “Adjust diameter..”.
// [7GUIs] Clicking on this entry will open another frame with a slider inside that adjusts the diameter of C.
// [7GUIs] Changes are applied immediately.
// [7GUIs] Closing this frame will mark the last diameter as significant for the undo/redo history.
// [7GUIs] Clicking undo will undo the last significant change (i.e. circle creation or diameter adjustment).
// [7GUIs] Clicking redo will reapply the last undoed change unless new changes were made by the user in the meantime.// [7GUIs] 

// [7GUIs] Circle Drawer’s goal is, among other things, to test how good the common challenge of implementing an undo/redo functionality for a GUI application can be solved.
// [7GUIs] In an ideal solution the undo/redo functionality comes for free resp. just comes out as a natural consequence of the language / toolkit / paradigm.
// [7GUIs] Moreover, Circle Drawer tests how dialog control*, i.e. keeping the relevant context between several successive GUI interaction steps, is achieved in the source code.
// [7GUIs] Last but not least, the ease of custom drawing is tested.// [7GUIs] 

// [7GUIs] * Dialog control is explained in more detail in the paper Developing GUI Applications: Architectural Patterns Revisited starting on page seven.
// [7GUIs] The term describes the challenge of retaining context between successive GUI operations.