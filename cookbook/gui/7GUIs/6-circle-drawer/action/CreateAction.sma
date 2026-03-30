use core
use base

_define_
CreateAction (Process view_)
{
    Spike undo
    Spike redo
    Spike del

    view aka view_

    Component hidden (1)

    undo -> (this) {
        root = find(this, "//")
        remove this.view from root.canvas.items
        addChildrenTo this.hidden {
            this.view
        }
    }

    redo -> (this) {
        root = find(this, "//")
        remove this.view from this.hidden
        addChildrenTo root.canvas.items {
            this.view
        }
    }

    del -> (this) {
        // delete only if an undo is active
        remove this.view from this.hidden
        delete this.view
    }

}
