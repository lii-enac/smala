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
    Scaling scaling (1, 1, 0, 0)
    pz1.zoom => scaling.sx, scaling.sy

    Translation translation (0, 0)
    pz1.xpan => translation.tx
    pz1.ypan => translation.ty

    Rectangle r (100, 100, 100, 100, 0, 0)

    OutlineWidth ow (3)
    3 / pz1.zoom => ow.width
    Circle c (300, 250, 50)

    FillColor fc(0, 0, 0)
    Text ts (200, 200, "this text size should scale")
  }

  Component zoomCoordOnlyScene {
    FillColor fc(0, 0, 0)
    Double x(50)
    Double y(400)
    Translation t (0, 0)
    (pz1.xpan + x) * pz1.zoom => t.tx
    (pz1.ypan + y) * pz1.zoom => t.ty
    Text te(0, 0, "this text position should move according to zoom, but the size should not scale")
  }

  Component stickToScreenScene {
    FillColor fc(0,0,0)
    Text t(0,20, "this text should stick to screen")
  }

  // debug
  //"yes" =: pz1.debugScene.state
}

run root
run syshook
