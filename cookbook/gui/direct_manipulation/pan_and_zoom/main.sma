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
*    Stephane Conversy <stephane.conversy@enaf.fr>
*
*/

use core
use base
use gui

import PanAndZoom

_main_
Component root {

  Frame f ("Pan and zoom", 0, 0, 800, 800)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1

  PanAndZoom pz1 (f)

  Component zoomScene {
    // Pan and zoom
    Scaling scaling (1, 1, 0, 0)
    Translation translation (0, 0)
    pz1.zoom => scaling.sx, scaling.sy
    pz1.xpan => translation.tx
    pz1.ypan => translation.ty

    // Graphics

    FillColor _ (70, 70, 70)
    Text ts (200, 200, "The scene can be panned and zoomed...")

    FillColor _ (200, 200, 200)
    FillOpacity _ (0.5)
    OutlineColor _ (70, 70, 70)
    Rectangle rectangle (100, 100, 100, 100, 0, 0)
  }

  Component zoomCoordOnlyScene {
    // positionning
    Double x (100)
    Double y (300)
    Translation t (0, 0)
    (pz1.xpan + x) * pz1.zoom => t.tx
    (pz1.ypan + y) * pz1.zoom => t.ty

    // Graphics

    FillColor _ (125, 125, 125)
    Text te (0, 0, "...except this text wich moves as a consequence of pan and zoom, but does not scale")
  }

  Component stickToScreenScene {
    FillColor _ (0, 0, 0)
    Text t (100, 400, "... and this text which is not transformed at all")
  }

  // debug
  //"yes" =: pz1.debugScene.state
}

run root
run syshook
