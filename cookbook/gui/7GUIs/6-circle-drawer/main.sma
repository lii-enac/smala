// [7GUIs] Circle Drawer
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#CRUD
// [7GUIs] Challenges: undo/redo, custom drawing, dialog control.

use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton
import gui.widgets.SimpleMenu
import gui.widgets.StandAloneSlider

import action.UndoRedoManager
import action.CreateAction
import action.ChangeRadiusAction

import CircleItem

_main_
Component root {
    Frame f ("7GUIs Circle Drawer", 500, 500, 360, 320)  // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop
    mouseTracking = 1 // for enter and leave events FIXME should be automatic

    int pushbutton_y = 20
    int pushbutton_offset = 5
    int canvas_offset = 12
    StandAlonePushButton undo("Undo", 0, 20)                        // [7GUIs] ...an undo and...
    StandAlonePushButton redo("Redo", 0, 20)                        // [7GUIs] ...redo button... 
    (f.width / 2) - undo.width - pushbutton_offset =:> undo.x
    (f.width / 2) + pushbutton_offset =:> redo.x

    // context 
    Double old_radius (0)

    Component canvas {  // [7GUIs] ...as well as a canvas area underneath.
        Int canvas_y (0)
        pushbutton_y + undo.height + canvas_offset =: canvas_y
        Translation t (0, 0)
        t.tx = canvas_offset
        canvas_y =: t.ty
        FillColor _ (#535353)
        RectangleClip clip (0, 0, 0, 0)
        Rectangle mask (0, 0, 0, 0, 0, 0 )
        f.width - (2* canvas_offset) =:> mask.width, clip.width
        f.height - canvas_y - canvas_offset =:> mask.height, clip.height
        List items                      
    }


    SimpleMenu menu (100, 100)                     // [7GUIs] a popup menu...
    addChildrenTo menu.choices {
        String _ ("Adjust diameter")             // [7GUIs] ... with one entry “Adjust diameter..”.
    }

    Ref _selected_item_ref(nullptr)
    DerefDouble _radius_of_selected_item (_selected_item_ref, "c/r", DJNN_GET_ON_CHANGE)

    UndoRedoManager undo_redo_manager               // FIXME should be provided in smala lib
    
    undo.click -> undo_redo_manager.undo            // [7GUIs] Clicking undo will undo the last significant change (i.e. circle creation or diameter adjustment).
    redo.click -> undo_redo_manager.redo            // [7GUIs] Clicking redo will reapply the last undoed change unless new changes were made by the user in the meantime.


    Component popup_slider (1) {   // permet de controler tout el composant popup 
        Frame f2 ("", 530, 700, 300, 100)
        // f2.close ->! popup_slider
        FillColor _ (White)
        Text t (50, 22, "Ajust diameter of circle at (x, y).")
        
        StandAloneSlider radius_slider(45, 25, 208, 10)    // [7GUIs] ...a slider inside that adjusts the diameter of C.
        

        // TextPrinter titi
        // "new radius : " + _radius_of_selected_item.value => titi.input
        
        radius_slider.output -> (root) {                 // [7GUIs] Clicking on this entry will open another frame with a slider inside that adjusts the diameter of C.
            selected_item = getRef(&root._selected_item_ref)
            if (&selected_item != null) {
                selected_item.c.r = $root.popup_slider.radius_slider.output // [7GUIs] Changes are applied immediately. 
            }
        }

        // FIXME : TODO SHOULD BE only once ! Save the previous radius as soon as the thumb is hovered.
        // radius_slider.button_fsm.hover -> (root) {
        //     selected_item = getRef(&root._selected_item_ref)
        //     root.popup_slider.old_radius = $selected_item.c.r
        // }

        // FIXME : TODO should be done on 'close', here we do it when the interaction is terminated but OLD radius should be the first one
        // radius_slider.button_fsm.idle -> (root) {       // [7GUIs] Closing this frame will mark the last diameter as significant for the undo/redo history.
        //     selected_item = getRef(&root._selected_item_ref)
        //     if (&selected_item != null && $selected_item.c.r != $root.popup_slider.old_radius) {
        //         // FIXME should be done by undo_redo_manager(?)
        //         activate (root.undo_redo_manager.removeActionsStartingFromCurrent) 
        //         graph_exec ()
        //         Process ca_ = ChangeRadiusAction (root.undo_redo_manager.actions, "cra_", selected_item, $root.popup_slider.old_radius, $selected_item.c.r)
        //     }
        //     print ("==== RADIUS ACTION ")
        //     dump root.undo_redo_manager.actions
        //     dump root.canvas.items
        // }    
    }

    _radius_of_selected_item.value => popup_slider.radius_slider.value


    (menu.selected == "Adjust diameter") -> pre_popup_slider : (root) {
        selected_item = getRef(&root._selected_item_ref)
        root.old_radius = $selected_item.c.r
    }
    pre_popup_slider -> popup_slider

    popup_slider.f2.close -> post_popup_slider : (root) {
        // print ("==== RADIUS ACTION ")
        selected_item = getRef(&root._selected_item_ref)
        if (&selected_item != null && $selected_item.c.r != $root.old_radius) {
            // FIXME should be done by undo_redo_manager(?)
            activate (root.undo_redo_manager.removeActionsStartingFromCurrent) 
            graph_exec ()
            Process ca_ = ChangeRadiusAction (root.undo_redo_manager.actions, "cra_", selected_item, $root.old_radius, $selected_item.c.r)
        }
        print ("==== RADIUS ACTION ")
        dump root.undo_redo_manager.actions
        dump root.canvas.items
    } 
    post_popup_slider ->! popup_slider


    //(menu.selected == "Adjust diameter") -> popup_slider
    //canvas.mask.left.press ->! popup_slider

    canvas.mask.left.press -> (root) {             // [7GUIs] Left-clicking inside an empty area inside the canvas will create...
        setRef (root._selected_item_ref, null) // safety ou useless - need to be tested ?
        addChildrenTo root.canvas.items {
            CircleItem selected_item (root, $root.canvas.mask.press.local_x, $root.canvas.mask.press.local_y ) // [7GUIs] ...circle with a fixed diameter whose center is the left-clicked point.
            setRef(root._selected_item_ref, &selected_item)
        }
        
        // FIXME should be done by undo_redo_manager(?)
        activate (root.undo_redo_manager.removeActionsStartingFromCurrent) 
        graph_exec ()
        Process ca_ = CreateAction (root.undo_redo_manager.actions, "ca_", getRef(root._selected_item_ref))

        print ("==== CREATE ACTION ")
        dump root.undo_redo_manager.actions
        dump root.canvas.items
    }
}




// [7GUIs]  The task is to build a frame containing an undo and redo button as well as a canvas area underneath.
// [7GUIs]  Left-clicking inside an empty area inside the canvas will create an unfilled circle with a fixed diameter whose center is the left-clicked point.
// [7GUIs]  The circle nearest to the mouse pointer such that the distance from its center to the pointer is less than its radius, if it exists, is filled with the color gray.
// [7GUIs]  The gray circle is the selected circle C. Right-clicking C will make a popup menu appear with one entry “Adjust diameter..”.
// [7GUIs]  Clicking on this entry will open another frame with a slider inside that adjusts the diameter of C.
// [7GUIs]  Changes are applied immediately.
// [7GUIs]  Closing this frame will mark the last diameter as significant for the undo/redo history.
// [7GUIs]  Clicking undo will undo the last significant change (i.e. circle creation or diameter adjustment).
// [7GUIs]  Clicking redo will reapply the last undoed change unless new changes were made by the user in the meantime.// [7GUIs] 

// [7GUIs] Circle Drawer’s goal is, among other things, to test how good the common challenge of implementing an undo/redo functionality for a GUI application can be solved.
// [7GUIs] In an ideal solution the undo/redo functionality comes for free resp. just comes out as a natural consequence of the language / toolkit / paradigm.
// [7GUIs] Moreover, Circle Drawer tests how dialog control*, i.e. keeping the relevant context between several successive GUI interaction steps, is achieved in the source code.
// [7GUIs] Last but not least, the ease of custom drawing is tested.// [7GUIs] 

// [7GUIs] * Dialog control is explained in more detail in the paper Developing GUI Applications: Architectural Patterns Revisited starting on page seven.
// [7GUIs] The term describes the challenge of retaining context between successive GUI operations.