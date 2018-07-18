use core
use base
use gui

_main_
Component root
{
	Frame f ("my frame", 0, 0, 400, 600)
  	//mouseTracking = 1

  	Circle mobile(100, 100, 40)

  	f.move.x => mobile.cx
  	f.move.y => mobile.cy
}

run root
run syshook
