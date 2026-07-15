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
