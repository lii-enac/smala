use core
use base
use display
use gui

import FittsTask1D

_main_
Component root
{
	Frame f ("my frame", 0, 0, 800, 800)
  Exit ex (0, 1)
  f.close -> ex
  	mouseTracking = 1

  	// Typical CHIesque Fitts (not the genuine, reciprocal from the 50's)
  	FittsTask1D fitts(f)

    Int target_width(16)
    Int target_distance(512)

    target_distance =: fitts.target_distance

    // classic
    target_width =:> fitts.target_width

  	// MacGuffin & Balakrishnan, 2002
    /*  	
    f.move.x > 0.9*fitts.target_distance
  		? target_width*2
  		: target_width
  	=:> fitts.target_width
  	*/
  	
  	// Zhai, Conversy, Guiard & Beaudouin-Lafon, 2003
    /*
  	Double
  	scale(0.5) // FIXME .5 does not work!!
  	//scale(1)
  	//scale(2)
  	// should be random

	  f.move.x > 0.9*fitts.target_distance
  		? target_width*scale
  		: target_width
  	=:> fitts.target_width
  	*/

    // debug result
    TextPrinter tp
    fitts.target_time_acquisition => tp.input
}

