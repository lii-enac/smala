use core
use base
use gui

_main_
Component root
{
<<<<<<< HEAD
	Frame f ("my frame", 0, 0, 800, 800)
=======
	Frame f ("my frame", 0, 0, 400, 600)
    // mouseTracking deactivated by default
>>>>>>> cookbook/gui - added comments in simplest cookbook
  	//mouseTracking = 1

	OutlineWidth ow(10)
	FillColor fc(255,0,0)
  	Circle mobile(100, 100, 40)

  	// Two behaviors:
  	//   mouseTracking activated
  	//     f.move is updated constantly so mobile follows mouse
  	//   mouseTracking deactivated
  	//     f.move behaves like f.press so mobile is moved only on mouse press
  	f.move.x => mobile.cx
  	f.move.y => mobile.cy



  	Polygon p {
  		Point p1(10,0)
  		Point p2(110,50)
  		Point p3(10,100)
  		/*Point _(867.,          256.        )
 		Point _(945.98376465,  385.31274414)
 		Point _(798.5927124,   350.15460205)
 		Point _(700.01623535,  465.23242188)
 		Point _(687.9072876,   314.19073486)
 		Point _(548.,          256.        )
 		Point _(687.9072876,   197.80926514)
 		Point _(700.01623535,   46.76756668)
 		Point _(798.5927124,   161.84539795)
 		Point _(945.98376465,  126.68724823)*/
  	}
}

run root
run syshook
