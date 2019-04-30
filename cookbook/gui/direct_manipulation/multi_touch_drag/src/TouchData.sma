use core
use base

_define_
TouchData ()
{
    Spike pressed
    Component press {
        Double x (0)
        Double y (0)
        //Double pressure (1)
    }

    Spike moved
    Component move {
        Double x (0)
        Double y (0)
        //Double pressure (1)
    }

    Spike released
    Component release {
        Double x (0)
        Double y (0)
        //Double pressure (1)
    }
}