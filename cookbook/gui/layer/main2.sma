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

//import gui.interactors.PanAndZoom
import PanAndZoom


_main_
Component root {
  Frame frame ("layer", 0, 0, 600, 600)
  Exit ex (0, 1)
  frame.close -> ex

  PanAndZoom pz (frame.move, frame.press, frame.release, frame.wheel.dy)

  Scaling zoom_tr (1,1, 0,0)
  Translation pan_tr (0,0)

  pz.zoom =:> zoom_tr.sx, zoom_tr.sy
  pz.{xpan, ypan} =:> pan_tr.{tx, ty}
  
  OutlineWidth ow (10)
  FillColor fc (255,0,0)
  OutlineColor _ (0,0,255)

  //Component bg { // classic group
  Layer bg { // complete window
  //Layer bg (0,0,600,600) { // complete window
  //Layer bg (0,0,600,585) { 
  //Layer bg (0,0,600,130) { // only height
  //Layer bg (0,110,600,30) { // this will slighty cut the stars at the bottom and at the top
  //Layer bg (45,110,500,30) { // this will slighty cut the stars at the bottom, the top, left and right
  
    Text _(0,20, "this text and 5000 stars are in a layer, while the circle is moving on top")
    Text _(0,30, "a clock applies every 2s a small x translation on the star and thus invalidates the layer")
    Translation t(0,0)
    for(int i=0; i<1; i++) {
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

  FSM DamageLayer {
    State idle
    State waiting {
      Timer t (1000)
      t.end -> bg.damaged
    }
    idle -> waiting (pz.zoom)
    idle -> waiting (pz.xpan)
    waiting -> waiting (pz.zoom, waiting.t.reset)
    waiting -> waiting (pz.xpan, waiting.t.reset)
    waiting -> idle (waiting.t.end)
  }

  TextPrinter tp
  DamageLayer.state =:> tp.input
  
  Rectangle _(0,0,1,1,0,0)

  //Translation t(0,0)
  //Circle _(0,0, 50)
  mouseTracking = 1
  //frame.move.x =:> t.tx
  //frame.move.y =:> t.ty

  Clock cl(2000)
  //cl.tick -> { 10 + bg.t.tx =: bg.t.tx }
}