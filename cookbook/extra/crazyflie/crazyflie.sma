use core
use base
use display
use gui
use extra

import Button


_main_
Frame f ("crazyflie", 0, 0, 600, 600)
Exit ex(0,1)
f.close -> ex

CrazyflieRC drone("radio://0/80/2M/E7E7E7E7E7")

FillColor _(0,0,255)

Button takeoff(f, "takeoff", 10, 10)
takeoff.click -> drone.takeoff

Button land(f, "land", 10, 100)
land.click -> drone.land
