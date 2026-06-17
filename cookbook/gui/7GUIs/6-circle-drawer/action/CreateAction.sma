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
        canvas = find(root, "//canvas")
        remove this.view from canvas.items
        addChildrenTo this.hidden {
            this.view
        }
    }

    redo -> (this) {
        root = find(this, "//")
        canvas = find(root, "//canvas")
        remove this.view from this.hidden
        addChildrenTo canvas.items {
            this.view
        }
    }

    del -> (this) {
        // delete only if an undo is active
        remove this.view from this.hidden
        delete this.view
    }

}
