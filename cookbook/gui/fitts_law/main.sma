use core
use base
use gui

import FittsTask1D

_main_
Component root
{
	Frame f ("my frame", 0, 0, 800, 800)
  	mouseTracking = 1

  	FittsTask1D fitts(f)
  	50 =: fitts.target_width
  	f.width/2 =: fitts.target_distance
}

run root
run syshook
