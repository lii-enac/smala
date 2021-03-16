/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2021)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use display
use gui

_main_
Component root {
  Frame f ("layer", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  OutlineWidth ow(10)
  FillColor fc(255,0,0)
  OutlineColor _(0,0,255)

  //Component bg {
  Layer bg {
    Text _(0,20, "this text and 5000 stars are in a layer, while the circle is moving on top")
    Text _(0,30, "a clock applies every 2s a small x translation on the star and thus invalidates the layer")
    Translation t(0,0)
    for(int i=0; i<5000; i++) {
      Component _ {
        Translation _(i/10.0,0)
        Translation t(-50,100)
        Scaling _(0.1,0.1,0,0)
        Polygon _ {
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
      }
    }
  }

  Translation t(0,0)
  Circle _(0,0, 50)

  mouseTracking = 1
  f.move.x =:> t.tx
  f.move.y =:> t.ty

  Clock cl(2000)
  cl.tick -> { 10 + bg.t.tx =: bg.t.tx }
}