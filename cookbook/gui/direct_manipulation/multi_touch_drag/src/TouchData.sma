use core
use exec_env
use base

_define_
TouchData ()
{
    Int touchId (0)

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