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
  // 255 =: frame.background_color.r
  // 255 =: frame.background_color.g
  // 255 =: frame.background_color.b

  PanAndZoom pz (frame.move, frame.press, frame.release, frame.wheel.dy)

  
  //Image _("mire.png", 1,1, -1,-1)
  //Image _("parrot.png", 1,10, -1,-1)

  Scaling zoom_tr (1,1, 0,0)
  Translation pan_tr (0,0)

  pz.zoom =:> zoom_tr.sx, zoom_tr.sy
  pz.{xpan, ypan} =:> pan_tr.{tx, ty}
  
  OutlineWidth ow (10)
  FillColor fc (255,0,0)
  OutlineColor _ (0,0,255)

  //Rotation _(30, 0,0)
  
  //Component bg { // classic group
  //  Bool damaged(1)
  Layer bg { // complete window
  //Layer bg (200) { // complete window with padding
  //Layer bg (0,0,600,600) { // complete window
  //Layer bg (0,0,600,585) { 
  //Layer bg (0,0,600,130) { // only height
  //Layer bg (0,110,600,30) { // this will slighty cut the stars at the bottom and at the top
  //Layer bg (45,110,500,30) { // this will slighty cut the stars at the bottom, the top, left and right

    // Image _("mire.png", 1,10, -1,-1)
    FillOpacity _(0.5)
    Rectangle _(1,30, -50, 50, 0, 0)

    //Text _(0,20, "this")
    Text _(0,20, "this text and 5000 stars are in a layer, while the circle is moving on top")
    Text _(0,30, "every 2s a clock applies a small x translation on the stars and thus invalidates the layer")
    Text _(0,40, "during a pan and zoom, after 500ms of stillness, the layer is freezed")

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

  NoOutline _()
  FillOpacity _(1.0)
  FillColor _(255,255,0)
  //Rectangle _(5,50,10,10,0,0)
  Circle _(5,50,5)

  FSM DamageLayer {
    State idle
    State waiting {
      Timer t (500) // beware, make sure the time is greater than the time it takes to render a frame, otherwise the layer will constantly be damaged
      t.end -> bg.damaged
    }
    idle -> waiting (pz.zoom)
    idle -> waiting (pz.xpan)
    waiting -> waiting (pz.zoom, waiting.t.reset)
    waiting -> waiting (pz.xpan, waiting.t.reset)
    waiting -> idle (waiting.t.end)
  }

  // TextPrinter tp
  // DamageLayer.state =:> tp.input
  
  
  //Translation t(0,0)
  //Circle _(0,0, 50)
  // mouseTracking = 1
  //frame.move.x =:> t.tx
  //frame.move.y =:> t.ty

  Clock cl(2000)
  AssignmentSequence do_translate(1) {
    10 + bg.t.tx =: bg.t.tx
  }
  cl.tick -> do_translate
  do_translate -> bg.damaged

  /*AssignmentSequence seq(1) {
    pz.xpan + 50 =: pz.xpan
    pz.ypan + 50 =: pz.ypan
    pz.zoom * 2 =: pz.zoom
  }*/

  // Timer cl2(1000)
  // cl2.end -> seq
  // Timer cl2(2000)
  // cl2.end -> seq

  // TextPrinter tp
  // pz.xpan =:> tp.input
  // pz.ypan =:> tp.input

}