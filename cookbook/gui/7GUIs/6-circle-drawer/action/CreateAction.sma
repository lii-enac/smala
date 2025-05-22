use core
use base

_define_
CreateAction (Process view_)
{
    Spike undo
    Spike redo
    Spike del

    view aka view_

    Component hidden (1) {

    }

    undo -> (this) {
        root = find(this, "//")
        remove this.view from root.canvas
        addChildrenTo this.hidden {
            this.view
        }
    }

    redo -> (this) {
        root = find(this, "//")
        remove this.view from this.hidden
        addChildrenTo root.canvas {
            this.view
        }
    }

    del -> (this) {
        // delete only if an undo is active because view and model has been removed from root.model and root.gui
        // if "idle" or redo everything is managed as normal
        for v: this.hidden {
            root = find(this, "//")
            remove this.view from this.hidden
            delete this.view
            //delete_box_and_links (root, v) // delete_box_and_links manage a box (process) delete : link, arrow, the view and model
        }
        //print ("CreateAction::del")
    }

}
