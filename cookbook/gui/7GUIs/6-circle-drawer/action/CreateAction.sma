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
        print ("UNDO: move " + this.view + " from canvas.items to hidden")
        dump root.canvas.items
    }

    redo -> (this) {
        root = find(this, "//")
        remove this.view from this.hidden
        addChildrenTo root.canvas.items {
            this.view
        }
        print ("REDO: move " + this.view + " from hidden to canvas.items")
        dump root.canvas.items
    }

    del -> (this) {
        // delete only if an undo is active because view and model has been removed from root.model and root.gui
        // if "idle" or redo everything is managed as normal
        //for v: this.hidden {
            //root = find(this, "//")
            print ("CreateAction::del : " /*+ this.view*/)
            remove this.view from this.hidden
            delete this.view // Quest ce qu'il reste dans la ref ?
        //}
        //print ("CreateAction::del")
    }

}
