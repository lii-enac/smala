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
    NoOutline _
    NoFill _
    Rectangle bg (0, 0, 0, 0, 0, 0)
    f.width => bg.width
    f.height => bg.height
  }

  // A few arbitrary transforms
  Rotation arbitraryRotation (19, 0, 0)
  Translation arbitraryTranslation (56, 24)
  Scaling arbitraryScaling (1.2, 1.2, 0, 0)

  Component zoomScene {
    // Pan and zoom
    Scaling zoomTransform (1, 1, 0, 0)
    Translation panTransform (0, 0)
    PanAndZoom pz (f, backgroundForPan.bg)
    pz.zoom => zoomTransform.sx, zoomTransform.sy
    pz.xpan => panTransform.tx
    pz.ypan => panTransform.ty

    // Graphics

    FillColor _ (70, 70, 70)
    Text ts (200, 200, "The scene can be panned and zoomed (and the rectangle dragged)...")

    // Draggable rectangle
    FillColor _ (200, 200, 200)
    FillOpacity _ (0.5)
    OutlineColor _ (70, 70, 70)
    Translation dragTransform (0, 0)
    Rectangle rectangle (100, 100, 100, 100, 0, 0)

    Drag drag (f, rectangle)
    drag.tx => dragTransform.tx
    drag.ty => dragTransform.ty
  }
  pz aka zoomScene.pz

  Component zoomCoordOnlyScene {
    // positionning
    Double x (100)
    Double y (300)
    Translation t (0, 0)
    (pz.xpan + x) * pz.zoom => t.tx
    (pz.ypan + y) * pz.zoom => t.ty

    // Graphics
    FillColor _ (125, 125, 125)
    Text te (0, 0, "...except this text wich moves as a consequence of pan and zoom, but does not scale")
  }

  Component stickToScreenScene {
    FillColor _ (0, 0, 0)
    Text t (100, 400, "... and this text which is not transformed at all")
  }
}

run root
run syshook
