/*
*  Smala cookbook 6-circle-drawer
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2026)
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

import gui.widgets.PushButton
import gui.widgets.HBox
import gui.widgets.SimpleMenu
import gui.widgets.StandAloneSlider

import action.UndoRedoManager
import action.CreateAction
import action.ChangeRadiusAction

import CircleItem

_define_
CircleDrawer (Process _frame_width, Process _frame_height)
{
    int canvas_offset = 12

    HBox buttons {
        PushButton undo ("Undo") // [7GUIs] ...an undo and...
        PushButton redo ("Redo") // [7GUIs] ...redo button...
    }
    buttons.v_alignment = 0     // Align on top
    buttons.space = 20

    undo aka buttons.undo
    redo aka buttons.redo

    // context 
    Double ctx_old_radius (0)

    Component canvas {  // [7GUIs] ...as well as a canvas area underneath.
        
        Translation t (canvas_offset, 0)
        buttons.min_height + canvas_offset =:> t.ty
        
        FillColor _ (#535353)
        RectangleClip clip (0, 0, 0, 0)
        Rectangle mask (0, 0, 0, 0, 0, 0 )
        _frame_width - (2 * canvas_offset) =:> mask.width, clip.width
        _frame_height - (2 * canvas_offset) - buttons.min_height =:> mask.height, clip.height
        
        List items
    }


    SimpleMenu menu (100, 100)                 // [7GUIs] a popup menu...
    addChildrenTo menu.choices {
        String _ ("Adjust diameter")           // [7GUIs] ... with one entry “Adjust diameter..”.
    }

    Ref _selected_item_ref(nullptr)
    DerefDouble _radius_of_selected_item (_selected_item_ref, "c/r", DJNN_GET_ON_CHANGE)

    UndoRedoManager undo_redo_manager          // FIXME should be provided in smala lib
    
    undo.click -> undo_redo_manager.undo       // [7GUIs] Clicking undo will undo the last significant change (i.e. circle creation or diameter adjustment).
    redo.click -> undo_redo_manager.redo       // [7GUIs] Clicking redo will reapply the last undoed change unless new changes were made by the user in the meantime.

    canvas.mask.left.press -> na_canvas_pressed:(this) {         // [7GUIs] Left-clicking inside an empty area inside the canvas will create...
    
        // Properly deselect the previously selected item, if any
        old_selected_item = getRef(&this._selected_item_ref)
        if (&old_selected_item != null) {
            activate (old_selected_item.deselect)
        }

        addChildrenTo this.canvas.items {
            CircleItem selected_item (this, $this.canvas.mask.press.local_x, $this.canvas.mask.press.local_y ) // [7GUIs] ...circle with a fixed diameter whose center is the left-clicked point.
            setRef(this._selected_item_ref, &selected_item)
        }
        
        /// FIXME: This should be handled by the undo_redo_manager.
        // Not that simple: we need a hook on system creation so that when a new
        // action is added to the manager, the stack can be cleaned from the
        // current position to the end if necessary.
        // Also, the following graph_exec must run immediately BEFORE adding the action.
        activate (this.undo_redo_manager.removeActionsStartingFromCurrent) 
        graph_exec ()
        Process ca_ = CreateAction (this.undo_redo_manager.actions, "ca_", getRef(this._selected_item_ref))

    }

    // popup windows: use (1) or is_modal = 1 to initialize closed
    Component popup_slider (1) {
        Frame f2 ("", 530, 700, 300, 100)
        FillColor _ (White)
        Text t (50, 22, "Ajust diameter of circle at (x, y).") 
        StandAloneSlider radius_slider(45, 25, 208, 10)  // [7GUIs] ...a slider inside that adjusts the diameter of C.
        
        radius_slider.output -> (this) {                 // [7GUIs] Clicking on this entry will open another frame with a slider inside that adjusts the diameter of C.
            selected_item = getRef(&this._selected_item_ref)
            if (&selected_item != null) {
                selected_item.c.r = $this.popup_slider.radius_slider.output // [7GUIs] Changes are applied immediately. 
            }
        }
    }

    _radius_of_selected_item.value => popup_slider.radius_slider.value

    // store radius before opening the popup slider
    (menu.selected == "Adjust diameter") -> pre_popup_slider : (this) {
        selected_item = getRef(&this._selected_item_ref)
        this.ctx_old_radius = $selected_item.c.r
    }
    // then open popup
    pre_popup_slider -> popup_slider

    // [7GUIs]  Closing this frame will mark the last diameter as significant for the undo/redo history.
    popup_slider.f2.close -> post_popup_slider : (this) {
        
        selected_item = getRef(&this._selected_item_ref)
        if (&selected_item != null && $selected_item.c.r != $this.ctx_old_radius) {
            /// FIXME: This should be handled by the undo_redo_manager.
            // Not that simple: we need a hook on system creation so that when a new
            // action is added to the manager, the stack can be cleaned from the
            // current position to the end if necessary.
            // Also, the following graph_exec must run immediately BEFORE adding the action.
            activate (this.undo_redo_manager.removeActionsStartingFromCurrent) 
            graph_exec ()
            Process ca_ = ChangeRadiusAction (this.undo_redo_manager.actions, "cra_", selected_item, $this.ctx_old_radius, $selected_item.c.r)
        }
    }
    // .. then close the popup
    post_popup_slider ->! popup_slider
}