use core
use base
use display
use gui

import FittsTask1D

_main_
Component root
{
	Frame f ("my frame", 0, 0, 800, 800)
  	mouseTracking = 1

  	// Fitts-like (not the genuine from the 50's)
  	FittsTask1D fitts(f)
  	16 =: fitts.target_width
  	512 =: fitts.target_distance

  	// FIXME when the two are active (MB and ZCGM), there are 2 refresh!!!

  	/*
  	// MacGuffin & Balakrishnan
  	f.move.x > 0.9*fitts.target_distance
  		? 50*2
  		: 50
  	=> fitts.target_width
  	*/

  	/*
  	// Zhai, Conversy, Guiard & Beaudouin-Lafon
  	Double
  	scale(0.5) // FIXME .5 does not work!!
  	//scale(1)
  	//scale(2)
  	// should be random

	f.move.x > 0.9*fitts.target_distance
  		? 50*scale
  		: 50
  	=> fitts.target_width
  	*/

}

run root
run syshook
