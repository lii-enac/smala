use core
use base
use display
use gui

import mspf

_main_
Component root
{
	Frame f ("simplest", 0, 0, 512, 512)
  //mouseTracking = 1

	OutlineWidth ow(10)
	FillColor fc(255,0,0)
  OutlineColor _(0,0,255)
  
  Circle mobile(100, 100, 40)

  f.move.x =:> mobile.cx
  f.move.y =:> mobile.cy

  FillColor _(255,255,255)
  mspf _(f)
}

run root
run syshook


/*
    Translation _(-650,0)
    Translation t(0,0)

    Polygon p {
      
      
      //Point p1(20,20)
      //Point p2(110,50)
      //Point p3(20,120)

      
      Point _(867.,          256.        )
      Point _(945.98376465,  385.31274414)
      Point _(798.5927124,   350.15460205)
      Point _(700.01623535,  465.23242188)
      Point _(687.9072876,   314.19073486)
      Point _(548.,          256.        )
      Point _(687.9072876,   197.80926514)
      Point _(700.01623535,   46.76756668)
      Point _(798.5927124,   161.84539795)
      Point _(945.98376465,  126.68724823)
      
    }

    Path _ {
      PathMove _(100,200)
      PathCubic _(100,100, 200,500, 300,400)
    }

    f.move.x =:> t.tx
    f.move.y =:> t.ty
*/