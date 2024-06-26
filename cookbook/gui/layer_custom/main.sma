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

  Int fakedy (0)

  Bool is_control_key (0)
  GenericKeyboard.key\-pressed == DJN_Key_Control -> { 1 =: is_control_key }
  GenericKeyboard.key\-released == DJN_Key_Control -> { 0 =: is_control_key }

  PanAndZoom pz (frame.move, frame.press, frame.release, fakedy)

  
  //Image _("mire.png", 1,1, -1,-1)
  //Image _("parrot.png", 1,10, -1,-1)

  Scaling zoom (1,1, 0,0)
  Translation translate (0,0)
  Rotation rot (0, 0, 0)
  frame.width / 2 =:> rot.cx
  frame.height /2 =:> rot.cy

  Switch sw_control_dy (false) {
    Component true{
      frame.wheel.dy/100. +=> rot.a
    }
    Component false {
      frame.wheel.dy => fakedy
    }
  }
  is_control_key =:> sw_control_dy.state

  pz.zoom =:> zoom.sx, zoom.sy
  pz.{xpan, ypan} =:> translate.{tx, ty}

  
  OutlineWidth ow (10)
  FillColor fc (255,0,0)
  OutlineColor _ (0,0,255)

  // note:
  // Here the layer will be damaged by the counter rotation in : counter_text
  // so we have to disconnect the do_damage spike on rotation to recompute the 
  // layer at each step and replace it with a connector to act as a regular component.
  // This will result in losing the benefit of the layer 
  // in this particular case but keeping it for scale and translation.
  // see Spike do_damage under

  Component l {
      Rotation outer_inv_rot (0, 0, 0) // inverse
      Translation outer_inv_translate (0,0) // inverse
      Scaling outer_inv_zoom (1,1, 0,0) // inverse
      rot.{cx, cy} =:> outer_inv_rot.{cx, cy}

      Layer bg (200) { // complete window

        //debug
        // FillColor _ (Yellow)
        // Rectangle debug_pad (0, 0, 0, 0, 0, 0)
        // FillColor _ (Green)
        // Rectangle debug (0, 0, 0, 0, 0, 0) 

        Scaling inner_zoom (1, 1, 0, 0)
        Translation inner_translate (0, 0)
        Rotation inner_rot (0, 0, 0)
        rot.{cx, cy} =:> inner_rot.{cx, cy}

        Component debug_repere_300 {
          FillColor _ (Blue)
          OutlineColor _ (Red)
          Circle c (0, 0, 10)
          inner_rot.{cx, cy} =:> c.{cx, cy}
        }

        FillColor _ (Red)
        Rectangle _(20,20, 50, 50, 0, 0)

        Component mytext {
          FillColor _ (White)
          Text _(50,20, " USE control/Apple key + mouse wheel to rotate layer")
        }

        Component counter_text {
          FillColor _ (White)
          Rotation counter_rot (0, 0, 0)
          -inner_rot.a =:> counter_rot.a
          Text _ (50, 100, " this txt Should always be horizontal")
        }
        
        Translation t(0,0)
        for(int i=0; i<50; i++) {
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
      frame.width =:> bg.width
      frame.height =:> bg.height
      //debug
      // frame.width =:> bg.debug.width  
      // frame.height =:> bg.debug.height
      // frame.width + 2 * bg.pad =:> bg.debug_pad.width
      // frame.height + 2 * bg.pad =:> bg.debug_pad.height
      // bg.x - bg.pad =:> bg.debug_pad.x
      // bg.y - bg.pad =:> bg.debug_pad.y 

      NoOutline _()
      FillOpacity _(1.0)
      FillColor _(255,255,0)

      // read the note above !
      Spike do_damage
      translate.tx -> do_damage
      translate.ty -> do_damage
      zoom.sx -> do_damage
      zoom.sy -> do_damage
      // rot.a -> do_damage
      -rot.a =:> outer_inv_rot.a
      rot.a =:> bg.inner_rot.a

      FSM damageLayerFSM {
        State idle {
            
            -translate.tx =: outer_inv_translate.tx
            -translate.ty =: outer_inv_translate.ty
            1./zoom.sx =: outer_inv_zoom.sx
            1./zoom.sy =: outer_inv_zoom.sy
          
            zoom.sx =: bg.inner_zoom.sx
            zoom.sy =: bg.inner_zoom.sy
            translate.tx =: bg.inner_translate.tx
            translate.ty =: bg.inner_translate.ty
            
        }
        State waiting {
          Timer t (200) // beware, make sure the time is greater than the time it takes to render a frame, otherwise the layer will constantly be damaged
          t.end -> bg.damaged
          do_damage -> t.reset
        }
        idle -> waiting (do_damage)
        waiting -> idle (waiting.t.end)
      }


      //debug
      // bg.damaged -> (root) {
      //   print ("==> layer is damaged")
      // }

 }
  FillColor _ (Pink)
  Translation t(0,0)
  Circle _(0,0, 50)

}