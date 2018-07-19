/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Nicolas Saporito  <nicolas.saporito@enac.fr>
*    Stephane Conversy <stephane.conversy@enac.fr>
*
*/

use core
use base
use gui


import Drag
import PanAndZoom


_main_
Component root {

  Frame f ("Pan and zoom and drag", 0, 0, 800, 800)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1

  // background to track mouse presses outside draggable shapes in order to prevent pan/drag conflict
  // (listening f.press doesn't allow to differentiate press inside or outside shapes)
  Component backgroundForPan {
    NoOutline no
    NoFill nf
    Rectangle bg (0, 0, 0, 0, 0, 0)
    f.width => bg.width
    f.height => bg.height
  }

  // A few arbitrary transforms
  Rotation arbitraryRotation (19, 0, 0)
  Translation arbitraryTranslation (56, 24)
  Scaling arbitraryScaling (1.2, 1.2, 0, 0)

  Component zoomScene {
    // Pan and zoom management of zoomScene
    Scaling zoomTransform (1, 1, 0, 0)
    Translation panTransform (0, 0)
    PanAndZoom pz (f, backgroundForPan.bg)
    pz.zoom => zoomTransform.sx, zoomTransform.sy
    pz.xpan => panTransform.tx
    pz.ypan => panTransform.ty

    // Drag management of a rectangle
    Translation dragTransform (0, 0)
    Rectangle r (100, 100, 100, 150, 0, 0)
    Drag drag (f, r)
    drag.tx => dragTransform.tx
    drag.ty => dragTransform.ty

    // non draggable figure and text
    OutlineWidth ow (3)
    3 / pz.zoom => ow.width
    Circle c (255, 150, 50)

    FillColor fc (0, 0, 0)
    Text ts (100, 215, "this text size should scale")
  }
  pz aka zoomScene.pz

  Component zoomCoordOnlyScene {
    FillColor fc (0, 0, 0)
    Double x (50)
    Double y (400)
    Translation t (0, 0)
    (pz.xpan + x) * pz.zoom => t.tx
    (pz.ypan + y) * pz.zoom => t.ty
    Text te (0, 0, "this text position should move according to zoom, but the size should not scale")
  }

  Component stickToScreenScene {
    FillColor fc (0,0,0)
    Text t (0,20, "this text should stick to screen")
  }

  // debug
  //"yes" =: pz.debugScene.state
}

run root
run syshook
