use core
use base


_define_
ChangeRadiusAction (Process circle_, int old_radius_, int new_radius_)
{
    Spike undo
    Spike redo
    Spike del

    circle aka circle_
    Int old_radius(old_radius_)
    Int new_radius(new_radius_)
    
    undo ->set_undo: {
        old_radius =: circle.c.r
    }

    redo ->set_redo: {
        new_radius =: circle.c.r
    }

    // del -> set_del: (this) {
    //     print  ("ChangeRadiusAction::del")
    // }

}
