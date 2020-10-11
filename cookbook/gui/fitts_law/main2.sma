use core
use base
use display
use gui
use files

import FittsTask1D

_main_
Component root
{
	Frame f ("my frame", 0, 0, 800, 800)
  	Exit ex (0, 1)
  	f.close -> ex
  	mouseTracking = 1

  	FittsTask1D fitts(f)
	
    Int target_width(0)
    Int target_distance(64)

    target_distance =:> fitts.target_distance

	SwitchList tries {
		Component t1 {
			8 =: target_width
			128 =: target_distance
		}
		Component t2 {
			64 =: target_width
			512 =: target_distance
		}
	}
	fitts.control.init -> tries.next

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

    // display result

	String hit_or_miss ("")
	AssignmentSequence hm_as(1) {
		fitts.control.state =: hit_or_miss
	}
	fitts.control.hit -> hm_as
	fitts.control.miss -> hm_as

    TextPrinter tp
	AssignmentSequence out_as(1) {
    	tries.index + " " + hit_or_miss + " " + target_width + " " + target_distance + " " + fitts.target_time_acquisition =: tp.input
	}
	fitts.control.init -> out_as
}

