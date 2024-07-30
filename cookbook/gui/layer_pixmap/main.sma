/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use display
use gui

import gui.archi.PixmapLayer
import PanAndZoom

_main_
Component root {
  Frame frame ("layer", 0, 0, 600, 600)
  Exit ex (0, 1)
  frame.close -> ex

  Int fakedy (0)

  Bool is_control_key (0)
  GenericKeyboard.key\-pressed == DJN_Key_Control -> { 1 =: is_control_key }
  GenericKeyboard.key\-released == DJN_Key_Control -> { 0 =: is_control_key }

  PanAndZoom pz (frame.move, frame.press, frame.release, fakedy)

  Scaling scale (1,1, 0,0)
  Translation translate (0,0)
  Rotation rotation (0, 0, 0)
  300 =: rotation.cx
  300 =: rotation.cy

  Switch sw_control_dy (false) {
    Component true{
      frame.wheel.dy/100. +=> rotation.a
    }
    Component false {
      frame.wheel.dy => fakedy
    }
  }
  is_control_key =:> sw_control_dy.state

  pz.zoom =:> scale.sx, scale.sy
  pz.{xpan, ypan} =:> translate.{tx, ty}

  OutlineWidth ow (10)
  FillColor fc (255,0,0)
  OutlineColor _ (0,0,255)

  /*****  Component to draw in the PaxmapLayer ****/
  Component layerCanvas {

    Component reference {
      FillColor _ (Blue)
      OutlineColor _ (Red)
      Circle c (0, 0, 10)
      rotation.{cx, cy} =: c.{cx, cy}
    }

    FillColor _ (Red)
    Rectangle _(20,20, 50, 50, 0, 0)

    Component mytext {
      FillColor _ (White)
      Text _(50,20, " USE control/Apple key + mouse wheel to rotate layer")
    }
  }

  Component l {

    /*****  PixmapLayer ****/
    // if you do not have scale, translate, rotation use nullptr.
    PixmapLayer pixmapLayer (200, 200, frame, scale, translate, rotation, layerCanvas)

    // manage do_damage spike of the PixmapLayer
    translate.tx -> pixmapLayer.do_damage
    translate.ty -> pixmapLayer.do_damage
    scale.sx -> pixmapLayer.do_damage
    scale.sy -> pixmapLayer.do_damage
    rotation.a -> pixmapLayer.do_damage
  }

  FillColor _ (Pink)
  Translation t(0,0)
  Circle _(0,0, 50)
}